/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_slam.h"
#include "tagslam/pose_estimate.h"
#include "tagslam/tag.h"
#include "tagslam/yaml_utils.h"
#include "tagslam/rigid_body.h"
#include "tagslam/bag_sync.h"
#include <XmlRpcException.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <tf_conversions/tf_eigen.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <boost/range/irange.hpp>
#include <math.h>
#include <fstream>
#include <iomanip>
#include <functional>

namespace tagslam {
  using boost::irange;
  struct PoseError {
    PoseError(double r = 0, double t = 0) : rot(r), trans(t) {
    }
    double rot{0};
    double trans{0};
  };

  TagSlam::TagSlam(const ros::NodeHandle& pnh) :  nh_(pnh) {
  }

  TagSlam::~TagSlam() {
  }

  bool TagSlam::initialize() {
    double pixNoise;
    nh_.param<double>("corner_measurement_error", pixNoise, 2.0);
    nh_.param<int>("max_number_of_frames", maxFrameNum_, 1000000);
    nh_.param<bool>("write_debug_images", writeDebugImages_, false);
    nh_.param<double>("max_initial_reprojection_error", maxInitialReprojError_, 100.0);
    ROS_INFO_STREAM("setting pixel noise to: " << pixNoise);
    tagGraph_.setPixelNoise(pixNoise);
    cameras_ = Camera::parse_cameras(nh_);
    if (cameras_.empty()) {
      ROS_ERROR("no cameras found!");
      return (false);
    }
    if (!subscribe()) {
      return (false);
    }
    for (const auto &cam_idx: irange(0ul, cameras_.size())) {
      camOdomPub_.push_back(
        nh_.advertise<nav_msgs::Odometry>("odom/cam_" +
                                          std::to_string(cam_idx), 1));
    }

    readMeasurements("distance");
    readMeasurements("position");

    if (!readRigidBodies()) {
      return (false);
    }
    for (const auto &rb: dynamicBodies_) {
      bodyOdomPub_.push_back(
        nh_.advertise<nav_msgs::Odometry>("odom/body_" + rb->name, 1));
    }
    nh_.param<double>("init_body_pose_max_error", initBodyPoseMaxError_, 1e10);
    nh_.param<std::string>("fixed_frame_id", fixedFrame_, "map");
    double maxDegree;
    nh_.param<double>("viewing_angle_threshold", maxDegree, 45.0);
    viewingAngleThreshold_ = std::cos(maxDegree/180.0 * M_PI);
    // play from bag file if file name is non-empty
    std::string bagFile;
    nh_.param<std::string>("bag_file", bagFile, "");
    if (!bagFile.empty()) {
      playFromBag(bagFile);
      tagGraph_.printDistances();
      ros::shutdown();
    }
    return (true);
  }
  
  
  void TagSlam::readMeasurements(const std::string &type) {
    // read distance measurements
    XmlRpc::XmlRpcValue meas;
    nh_.getParam("tag_poses/" + type + "_measurements", meas);
    if (meas.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      size_t nfound = 0;
      if (type == "distance") {
        distanceMeasurements_ = DistanceMeasurement::parse(meas);
        unappliedDistanceMeasurements_ = distanceMeasurements_;
        nfound = distanceMeasurements_.size();
      } else if (type == "position") {
        positionMeasurements_ = PositionMeasurement::parse(meas);
        unappliedPositionMeasurements_ = positionMeasurements_;
        nfound = positionMeasurements_.size();
      }
      ROS_INFO_STREAM("found " << nfound << " " << type <<
                      " measurements!");
    } else {
      ROS_INFO_STREAM("no " << type << " measurements found!");
    }
  }


  bool TagSlam::readRigidBodies() {
    XmlRpc::XmlRpcValue bodies, body_defaults;
    nh_.getParam("tag_poses/bodies", bodies);
    nh_.getParam("tag_poses/body_defaults", body_defaults);
    nh_.param<std::string>("tag_poses_out_file", tagPosesOutFile_,
                           "poses_out.yaml");
    nh_.param<std::string>("tag_world_poses_out_file",
                           tagWorldPosesOutFile_,
                           "tag_world_poses_out.yaml");
    nh_.param<std::string>("measurements_out_file",
                           measurementsOutFile_,
                           "measurements.yaml");
    if (bodies.getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
      ROS_ERROR("cannot find bodies in yaml file!");
      return (false);
    }
    RigidBodyVec rbv = RigidBody::parse_bodies(body_defaults, bodies);
    ROS_INFO_STREAM("configured bodies: " << rbv.size());
    if (rbv.size() > tagGraph_.getMaxNumBodies()) {
      ROS_ERROR_STREAM("too many bodies, max is: "
                       << tagGraph_.getMaxNumBodies());
      return (false);
    }
    if (rbv.empty()) {
      ROS_ERROR("no rigid bodies found!");
      return (false);
    }
    // find all tags and split bodies into static/dynamic
    for (auto &rb: rbv) {
      ROS_INFO_STREAM((rb->isStatic ? "static " : "dynamic ") << rb->name
                      << " has tags: " << rb->tags.size());
      TagVec tvec;
      for (auto &t: rb->tags) {
        if (t.second->poseEstimate.isValid()) {
          if (!rb->isStatic || rb->poseEstimate.isValid()) {
            tvec.push_back(t.second);
            allTags_.insert(t);
          }
        }
      }
      tagGraph_.addTags(rb, tvec);
      allBodies_.push_back(rb);
      if (rb->isDefaultBody) {
        defaultBody_ = rb;
      }
      (rb->isStatic ? staticBodies_ : dynamicBodies_).push_back(rb);
    }
    return (true);
  }

  bool TagSlam::subscribe() {
    if (cameras_.size() == 1) {
      singleCamSub_ = nh_.subscribe(cameras_[0]->tagtopic, 1,
                                    &TagSlam::callback1, this);
    } else {
      for (const auto &cam : cameras_) {
        sub_.push_back(std::shared_ptr<TagSubscriber>(
                         new TagSubscriber(nh_, cam->tagtopic, 1)));
      }
      switch (cameras_.size()) {
      case 2:
        approxSync2_.reset(new TimeSync2(SyncPolicy2(60/*q size*/),
                                         *(sub_[0]), *(sub_[1])));
        approxSync2_->registerCallback(&TagSlam::callback2, this);
        break;
      case 3:
        approxSync3_.reset(
          new TimeSync3(SyncPolicy3(60/*q size*/),
                        *(sub_[0]), *(sub_[1]), *(sub_[2])));
        approxSync3_->registerCallback(&TagSlam::callback3, this);
        break;
      case 4:
        approxSync4_.reset(
          new TimeSync4(SyncPolicy4(60/*q size*/),
                        *(sub_[0]), *(sub_[1]), *(sub_[2]), *(sub_[3])));
        approxSync4_->registerCallback(&TagSlam::callback4, this);
        break;
      case 5:
        approxSync5_.reset(
          new TimeSync5(SyncPolicy5(60/*q size*/),
                        *(sub_[0]), *(sub_[1]), *(sub_[2]), *(sub_[3]),
                        *(sub_[4])));
        approxSync5_->registerCallback(&TagSlam::callback5, this);
        break;
      case 6:
        approxSync6_.reset(
          new TimeSync6(SyncPolicy6(60/*q size*/),
                        *(sub_[0]), *(sub_[1]), *(sub_[2]), *(sub_[3]),
                        *(sub_[4]), *(sub_[5])));
        approxSync6_->registerCallback(&TagSlam::callback6, this);
        break;
      case 7:
        approxSync7_.reset(
          new TimeSync7(SyncPolicy7(60/*q size*/),
                        *(sub_[0]), *(sub_[1]), *(sub_[2]), *(sub_[3]),
                        *(sub_[4]), *(sub_[5]), *(sub_[6])));
        approxSync7_->registerCallback(&TagSlam::callback7, this);
        break;
      case 8:
        approxSync8_.reset(
          new TimeSync8(SyncPolicy8(60/*q size*/),
                        *(sub_[0]), *(sub_[1]), *(sub_[2]), *(sub_[3]),
                        *(sub_[4]), *(sub_[5]), *(sub_[6]), *(sub_[7])));
        approxSync8_->registerCallback(&TagSlam::callback8, this);
        break;
      default:
        ROS_ERROR_STREAM("number of cameras too large: " << cameras_.size());
        return (false);
        break;
      }
    }
    ROS_INFO_STREAM("subscribed to " << cameras_.size() << " cameras");
    return (true);
  }

  static ros::Time get_latest_time(const std::vector<TagArrayConstPtr> &msgvec) {
   ros::Time t(0);
    for (const auto &m: msgvec) {
      if (m->header.stamp > t) t = m->header.stamp;
    }
    return (t);
  }

  static gtsam::Pose3
  to_gtsam(const cv::Mat &rvec, const cv::Mat &tvec) {
    gtsam::Vector tvec_gtsam = (gtsam::Vector(3) <<
                                tvec.at<double>(0),
                                tvec.at<double>(1),
                                tvec.at<double>(2)).finished();
    gtsam::Pose3 p(gtsam::Rot3::rodriguez(
                     rvec.at<double>(0),
                     rvec.at<double>(1),
                     rvec.at<double>(2)), tvec_gtsam);
    return (p);
  }

  static void
  from_gtsam(cv::Mat *rvec, cv::Mat *tvec, const gtsam::Pose3 &p) {
    const gtsam::Point3 rv = gtsam::Rot3::Logmap(p.rotation());
    const gtsam::Point3 tv = p.translation();
    *rvec = (cv::Mat_<double>(3,1) << rv.x(), rv.y(), rv.z());
    *tvec = (cv::Mat_<double>(3,1) << tv.x(), tv.y(), tv.z());
  }

  static void to_opencv(std::vector<cv::Point3d> *a,
                       const std::vector<gtsam::Point3> b) {
    for (const auto &p: b) {
      a->emplace_back(p.x(), p.y(), p.z());
    }
  }
  static void to_opencv(std::vector<cv::Point2d> *a,
                       const std::vector<gtsam::Point2> b) {
    for (const auto &p: b) {
      a->emplace_back(p.x(), p.y());
    }
  }

  PoseEstimate
  TagSlam::estimatePosePNP(int cam_idx,
                           const std::vector<gtsam::Point3>&wpts,
                           const std::vector<gtsam::Point2>&ipts) const {
    PoseEstimate pe;
    if (!ipts.empty()) {
      std::vector<cv::Point3d> wp;
      std::vector<cv::Point2d> ip;
      to_opencv(&wp, wpts);
      to_opencv(&ip, ipts);
      const auto   &ci  = cameras_[cam_idx]->intrinsics;
      cv::Mat rvec, tvec;
      bool rc = utils::get_init_pose_pnp(wp, ip, ci.K,
                                         ci.distortion_model,
                                         ci.D, &rvec, &tvec);
      if (rc) {
        const auto T_c_w = to_gtsam(rvec, tvec);
        pe = T_c_w.inverse();
        pe.setError(utils::reprojection_error(wp, ip, rvec, tvec, ci.K,
                                              ci.distortion_model, ci.D));
      }
    }
    return (pe);
  }
                              
  PoseEstimate
  TagSlam::estimatePoseHomography(int cam_idx,
                           const std::vector<gtsam::Point3>&wpts,
                           const std::vector<gtsam::Point2>&ipts) const {
    PoseEstimate pe;
    if (!ipts.empty()) {
      std::vector<cv::Point3d> wp;
      std::vector<cv::Point2d> ip;
      to_opencv(&wp, wpts);
      to_opencv(&ip, ipts);
      const auto   &ci  = cameras_[cam_idx]->intrinsics;
      cv::Mat rvec, tvec;
      bool rc = utils::get_init_pose(wp, ip, ci.K,
                                     ci.distortion_model,
                                     ci.D, &rvec, &tvec);
      if (rc) {
        const auto T_c_w = to_gtsam(rvec, tvec);
        pe = T_c_w.inverse();
        pe.setError(utils::reprojection_error(wp, ip, rvec, tvec, ci.K,
                                              ci.distortion_model, ci.D));
      }
    }
    return (pe);
  }
                              

  void TagSlam::updatePosesFromGraph(unsigned int frame) {
    for (const auto &cam: cameras_) {
      cam->poseEstimate = tagGraph_.getCameraPose(cam, frame);
    }
    for (const auto &rb: allBodies_) {
      PoseEstimate pe;
      if (tagGraph_.getBodyPose(rb, &pe, frame)) {
        //std::cout << "UPDATE: body " << rb->name << " from " << std::endl;
        //std::cout << rb->poseEstimate.getPose() << std::endl << " to: " << std::endl;
        
        //rb->poseEstimate.setPose(p);
        rb->poseEstimate = pe;
        //std::cout << rb->poseEstimate << std::endl;
      } else {
        if (!rb->isStatic) {
          // mark pose estimate of dynamic bodies as invalid
          // because it will change from frame to frame
          rb->poseEstimate = PoseEstimate();//invalid
        }
      }
      if (rb->poseEstimate.isValid()) {
        for (auto &t: rb->tags) {
          TagPtr tag = t.second;
          gtsam::Pose3 pose;
          if (tagGraph_.getTagRelPose(rb, tag->id, &pose)) {
            //std::cout << "UPDATE: tag id " << tag->id << " from: " << std::endl <<
            //tag->poseEstimate.getPose()  << std::endl << " to: " << std::endl <<  pose << std::endl;
            tag->poseEstimate = PoseEstimate(pose, 0.0, 0);
          } else {
            tag->poseEstimate = PoseEstimate(); // invalid
          }
        }
      }
    }
  }

  unsigned int TagSlam::attachObservedTagsToBodies(
    const std::vector<TagArrayConstPtr> &msgvec) {
    unsigned int ntags(0);
    for (const auto cam_idx: irange(0ul, msgvec.size())) {
      const auto &tags = msgvec[cam_idx];
      for (auto &body : allBodies_) {
        ntags += body->attachObservedTags(cam_idx, tags);
      }
    }
    return (ntags);
  }

  RigidBodyPtr
  TagSlam::findBodyForTag(int tagId, int bits) const {
    for (auto &body : allBodies_) {
      if (body->hasTag(tagId, bits)) return (body);
    }
    return (NULL);
  }

  void TagSlam::discoverTags(const std::vector<TagArrayConstPtr> &msgvec) {
    for (const auto cam_idx: irange(0ul, msgvec.size())) {
      const auto &tags = msgvec[cam_idx];
      for (const auto &t: tags->apriltags) {
        if (allTags_.count(t.id) == 0) {
          // found new tag
          RigidBodyPtr body = findBodyForTag(t.id, t.bits);
          if (!body) {
            body = defaultBody_;
          }
          if (body) {
            TagPtr tag = body->addDefaultTag(t.id, t.bits);
            ROS_INFO_STREAM("found new tag: " << tag->id
                            << " of size: " << tag->size
                            << " for body: " << body->name);
            allTags_.insert(IdToTagMap::value_type(tag->id, tag));
          }
        }
      }
    }
  }

//#define DEBUG_POSE_ESTIMATE
  PoseEstimate
  TagSlam::poseFromPoints(int cam_idx,
                          const std::vector<gtsam::Point3> &wp,
                          const std::vector<gtsam::Point2> &ip,
                          bool pointsArePlanar) const {
#ifdef DEBUG_POSE_ESTIMATE
    std::cout << "------ points for camera pose estimate:------" << std::endl;
    for (const auto i: irange(0ul, wp.size())) {
      std::cout << cam_idx << " " << wp[i].x() << " " << wp[i].y() << " " << wp[i].z()
                << " " << ip[i].x() << " " << ip[i].y() << std::endl;
    }
    std::cout << "------" << std::endl;
#endif
    PoseEstimate pe = estimatePosePNP(cam_idx, wp, ip);
    std::cout << "pnp pose estimate has error: " << pe.getError() << std::endl;
    if (!pe.isValid()) {
      // PNP failed, try with a local graph
      const PoseEstimate &prevPose = cameras_[cam_idx]->poseEstimate;
      //std::cout << "running mini graph for cam " << cam_idx << " prev pose: " << std::endl << prevPose << std::endl;
      pe = initialPoseGraph_.estimateCameraPose(cameras_[cam_idx], wp, ip,
                                                prevPose);
      std::cout << "mini graph estimate err: " << pe.getError() << std::endl;
      if (pe.getError() > maxInitialReprojError_) {
        ROS_WARN_STREAM("cannot find pose of camera " <<
                        cam_idx << " err: " << pe.getError());
        pe.setValid(false);
      }
    }
    return (pe);
  }

  PoseEstimate
  TagSlam::findCameraPose(int cam_idx, const RigidBodyConstVec &rigidBodies,
                          bool bodiesMustHavePose) const {
    std::vector<gtsam::Point3> wp;
    std::vector<gtsam::Point2> ip;
    for (const auto &rb : rigidBodies) {
      if (!bodiesMustHavePose || rb->poseEstimate.isValid()) {
        rb->getAttachedPoints(cam_idx, &wp, &ip);
      }
    }
    if (!wp.empty()) {
      return (poseFromPoints(cam_idx, wp, ip));
    }
    return (PoseEstimate()); // invalid pose estimate
  }
                                       

  void TagSlam::findInitialCameraPoses() {
    for (const auto cam_idx: irange(0ul, cameras_.size())) {
      cameras_[cam_idx]->poseEstimate =
        findCameraPose(cam_idx, {staticBodies_.begin(), staticBodies_.end()}, true);
      //std::cout << "initial cam pose: " << cameras_[cam_idx]->poseEstimate << std::endl;
    }
  }

  bool TagSlam::isBadViewingAngle(const gtsam::Pose3 &p) const {
    double costheta = -p.matrix()(2,2);
    return (costheta < viewingAngleThreshold_);
  }
  
  bool
  TagSlam::estimateTagPose(int cam_idx,
                           const gtsam::Pose3 &bodyPose,
                           const TagPtr &tag) const {
    const auto &wp = tag->getObjectCorners();
    const auto ip = tag->getImageCorners();
    PoseEstimate pe = poseFromPoints(cam_idx, wp, ip, false);
    if (pe.isValid()) {
      if (isBadViewingAngle(pe.getPose())) {
        ROS_INFO_STREAM("IGNORING tag " << tag->id << " (bad viewing angle)");
        tag->poseEstimate = PoseEstimate(); // mark invalid
        return (false);
      }
      const CameraPtr &cam = cameras_[cam_idx];
      // pe pose estimate has T_o_c
      // body pose has T_w_b
      // camera pose has T_w_c
      // tag pose should have T_b_o
      // T_b_o = T_b_w * T_w_c * T_c_o
      gtsam::Pose3 pose = bodyPose.inverse() *
        cam->poseEstimate * pe.inverse();
      tag->poseEstimate = PoseEstimate(pose, 0.0, 0);
      //std::cout << "init tag pose est: " << tag->id << std::endl << " T_c_o: " << pe.inverse() << std::endl;
      //std::cout << "T_w_c: " << cam->poseEstimate.getPose() << std::endl;
      //std::cout << "T_b_w: " << bodyPose.inverse() <<  std::endl;
      //std::cout << "T_b_o: " << tag->poseEstimate.getPose() <<  std::endl;
    }
    //std::cout << "pose estimate for tag " << tag->id << ": " << pe << std::endl;
    return (true);
  }
                                
  void TagSlam::findInitialDiscoveredTagPoses() {
    for (auto &rb: allBodies_) {
      if (rb->poseEstimate.isValid()) {
        //std::cout << "RIGID BODY " << rb->name << " has pose: " << rb->poseEstimate << std::endl;
        // body has valid pose, let's see what
        // observed tags it has
        for (auto &tagMap: rb->observedTags) {
          const CameraPtr &cam = cameras_[tagMap.first];
          if (cam->poseEstimate.isValid()) {
            for (const auto &tag: tagMap.second) {
              auto gTagIt = allTags_.find(tag->id);
              if (gTagIt == allTags_.end()) {
                ROS_ERROR_STREAM("ERROR: invalid tag id: " << tag->id);
                continue;
              }
              TagPtr globalTag = gTagIt->second;
              if (!globalTag->poseEstimate.isValid()) {
                //std::cout << "tag: " << globalTag << " id " << globalTag->id << " has no valid pose!" << std::endl;
                if (estimateTagPose(tagMap.first, rb->poseEstimate.getPose(), tag)) {
                  TagVec tvec = {tag};
                  tagGraph_.addTags(rb, tvec);
                  globalTag->poseEstimate = tag->poseEstimate;
                }
              } else {
                tag->poseEstimate = globalTag->poseEstimate;
              }
            }
          }
        }
      }
    }
    // loop through all bodies and look for tags that
    // don't have a pose estimate yet. If body pose estimate is valid
    // and corresponding camera pose estimate is good too,
    // estimate the tag pose relative to body.
  }
        
  void TagSlam::processTags(const std::vector<TagArrayConstPtr> &msgvec) {
    // check if any of the tags are new, and associate them
    // with a rigid body
    discoverTags(msgvec);
    // Sort the tags according to which bodies they
    // belong to. The observed tags are then hanging
    // off of the bodies, to be used subsequently
    const auto nobs = attachObservedTagsToBodies(msgvec);
    // Go over all bodies and use tags with
    // established positions to determine camera poses
    findInitialCameraPoses();
    // For dynamic and pose-free static bodies, find their
    // initial poses if any of their tags are observed
    findInitialBodyPoses();
    // Any newly discovered tags can now be given
    // an initial pose, too.
    findInitialDiscoveredTagPoses();
    
    runOptimizer();
    updatePosesFromGraph(frameNum_);
    const auto distances = getDistances();
    const auto positions = getPositions();
    printDistanceErrors(distances);
    printPositionErrors(positions);
    computeProjectionError();
    detachObservedTagsFromBodies();

    ros::Time t = get_latest_time(msgvec);
    broadcastCameraPoses(t);
    broadcastBodyPoses(t);
    broadcastTagPoses(t);
    writeBodyPoses(tagPosesOutFile_);
    writeTagWorldPoses(tagWorldPosesOutFile_, frameNum_);
    writeMeasurements(measurementsOutFile_, distances, positions);
    ROS_INFO_STREAM("frame " << frameNum_ << " total tags: " << allTags_.size()
                    << " obs: " << nobs << " err: " << tagGraph_.getError()
                    << " iter: " << tagGraph_.getIterations());
    frameNum_++;
    std::cout << std::flush;
  }

  struct Stat {
    Stat(double s = 0, unsigned int c =0) :sum(s), cnt(c) {}
    Stat &operator+=(const Stat &b) {
      sum += b.sum;
      cnt += b.cnt;
      return (*this);
    }
    double sum{0};
    unsigned int cnt{0};
    double avg() const {
      return (cnt > 0 ? sqrt(sum/cnt) : 0.0);
    }
  };

  Stat operator+(const Stat &a, const Stat &b) {
    return (Stat(a.sum + b.sum, a.cnt + b.cnt));
  }


  void TagSlam::computeProjectionError() {
    std::vector<Stat> camStats(cameras_.size());
    std::vector<Stat> bodyStats(allBodies_.size());
    std::map<int, Stat> tagStats;
    std::map<double, std::pair<int, int>> sortedTagErrors;
    for (const auto cam_idx: irange(0ul, cameras_.size())) {
      const auto &cam = cameras_[cam_idx];
      if (!cam->poseEstimate.isValid()) {
        continue;
      }
      cv::Mat img;
      if (cam_idx < images_.size()) img = images_[cam_idx];
      cv::Mat rvec, tvec;
      from_gtsam(&rvec, &tvec, cam->poseEstimate.getPose().inverse());
      std::vector<Stat> bodyCamStats(allBodies_.size()); // per cam stat
      for (const auto body_idx: irange(0ul, allBodies_.size())) {
        const auto &rb = allBodies_[body_idx];
        if (!rb->poseEstimate.isValid()) {
          continue;
        }
        std::vector<gtsam::Point3> wpts;     // world points
        std::vector<gtsam::Point2> ipts;
        std::vector<int> tagids;
        rb->getAttachedPoints(cam_idx, &wpts, &ipts, &tagids);
        std::vector<cv::Point3d> wp;
        std::vector<cv::Point2d> ip, ipp;
        to_opencv(&wp, wpts);
        to_opencv(&ip, ipts);
        const auto &ci = cam->intrinsics;
        utils::project_points(wp, rvec, tvec, ci.K,
                              ci.distortion_model, ci.D, &ipp);
        if (img.rows > 0) {
          const cv::Scalar origColor(0,255,0), projColor(255,0,255);
          const cv::Size rsz(4,4);
          for (const auto i: irange(0ul, wp.size())) {
            cv::rectangle(img, cv::Rect(ip[i], rsz),  origColor, 2, 8, 0);
            cv::rectangle(img, cv::Rect(ipp[i], rsz), projColor, 2, 8, 0);
          }
        }

        for (const auto tag_idx: irange(0ul, tagids.size())) {
          Stat s(0, 4);
          for (const auto i: irange(0, 4)) {
            const cv::Point diff = ipp[tag_idx * 4 + i] - ip[tag_idx * 4 + i];
            s.sum += diff.x * diff.x + diff.y * diff.y;
          }
          // XXX will overwrite entry if error is identical!
          sortedTagErrors[s.avg()] = std::pair<int, int>(cam_idx, tagids[tag_idx]);
          tagStats[tagids[tag_idx]] += s;
          bodyCamStats[body_idx]    += s;
          bodyStats[body_idx]       += s;
          camStats[cam_idx]         += s;
        }
        if (!tagids.empty()) {
          ROS_INFO_STREAM("cam " << cam->name << " body: " << rb->name << " err: " << bodyCamStats[body_idx].avg());
        }
      }
      if (img.rows > 0 && writeDebugImages_) {
        std::string fbase = "image_" + std::to_string(frameNum_) + "_";
        cv::imwrite(fbase + std::to_string(cam_idx) + ".jpg", img);
      }

    }
    for (const auto cam_idx: irange(0ul, cameras_.size())) {
      const auto &cam = cameras_[cam_idx];
      if (camStats[cam_idx].cnt > 0) {
        ROS_INFO_STREAM("error for cam " << cam->name << ": " << camStats[cam_idx].avg());
      }
    }
    for (const auto body_idx: irange(0ul, allBodies_.size())) {
      const auto &rb = allBodies_[body_idx];
      if (bodyStats[body_idx].cnt > 0) {
        ROS_INFO_STREAM("error for body " << rb->name << ": " << bodyStats[body_idx].avg());
      }
    }
    for (const auto &ts: tagStats) {
      if (ts.second.cnt > 0) {
        ROS_INFO_STREAM("error for tag: " << ts.first << ": " << ts.second.avg());
      }
    }
    for (const auto &se: sortedTagErrors) {
      ROS_INFO_STREAM("error for cam: " << se.second.first << " tag: "
                      << se.second.second << " is: " << se.first);
    }
  }

  void TagSlam::detachObservedTagsFromBodies() {
    for (const auto &rb: allBodies_) {
      rb->detachObservedTags();
    }
  }

  void TagSlam::runOptimizer() {
    for (const auto &rb: allBodies_) {
      for (const auto &camToTag: rb->observedTags) {
        tagGraph_.observedTags(cameras_[camToTag.first],
                               rb, camToTag.second, frameNum_);
      }
    }
    tagGraph_.optimize();
  }

  PoseEstimate
  TagSlam::estimateBodyPose(const RigidBodyConstPtr &rb) const {
    //std::cout << "&&&& estimating body pose for " << rb->name << std::endl;
    PoseEstimate bodyPose; // defaults to invalid pose estimate
    int best_cam_idx = rb->bestCamera();
    if (best_cam_idx < 0) {
      // don't see any tags of the body in this frame!
      return (bodyPose);
    }
    RigidBodyConstVec rbv = {rb};
    PoseEstimate pe  = findCameraPose(best_cam_idx, rbv, false);
    // pose estimate has T_b_c
    // T_w_b = T_w_c * T_c_b
    const auto &T_w_c = cameras_[best_cam_idx]->poseEstimate;
    gtsam::Pose3 bPose = T_w_c * pe.inverse();
    //std::cout << "body pose from single camera " << best_cam_idx << std::endl;
    //std::cout << bPose << std::endl;

    bodyPose = initialPoseGraph_.estimateBodyPose(cameras_, images_, frameNum_, rb, bPose);
    //std::cout << "body pose from body graph: " << std::endl << bodyPose << std::endl;

    if (bodyPose.getError() > initBodyPoseMaxError_) {
      ROS_WARN_STREAM("no body pose for " << rb->name << " due to high error: " << bodyPose.getError());
      bodyPose.setError(1e10);
    }
    return (bodyPose);
  }

  void TagSlam::findInitialBodyPoses() {
    for (auto &rb: allBodies_) {
      if (rb->isStatic && rb->poseEstimate.isValid()) {
        continue;
      }
      rb->poseEstimate = PoseEstimate(); // mark invalid
      PoseEstimate pe = estimateBodyPose(rb);
      if (pe.isValid()) {
        rb->poseEstimate = pe;
        if (!rb->isStatic) {
          std::cout << "updated dynamic body pose for " << rb->name << " to " << rb->poseEstimate << std::endl;
        }
        if (rb->isStatic) {
          std::cout << "static body pose discovered for: " << rb->name << std::endl;
          // We encountered tags on a static body for the first time.
          // Any of these tags that have a known pose estimate can
          // be added to the graph and the global set of known tags.
          TagVec tvec;
          for (auto &t: rb->tags) {
            if (t.second->poseEstimate.isValid()) {
              tvec.push_back(t.second);
              allTags_[t.second->id] = t.second;
            }
          }
          tagGraph_.addTags(rb, tvec);
        }
      }
    }
    // Add new distance measurements if possible
    applyDistanceMeasurements();
    applyPositionMeasurements();
  }


  void TagSlam::applyDistanceMeasurements() {
    if (unappliedDistanceMeasurements_.empty()) {
      return;
    }
    DistanceMeasurementVec dmv;
    for (const auto &dm: unappliedDistanceMeasurements_) {
      bool used(false);
      if (allTags_.count(dm->tag1) > 0 &&
          allTags_.count(dm->tag2) > 0) {
        const auto tag1 = allTags_[dm->tag1];
        const auto tag2 = allTags_[dm->tag2];
        const auto rb1 = findBodyForTag(dm->tag1, tag1->bits);
        const auto rb2 = findBodyForTag(dm->tag2, tag2->bits);
        if (rb1 && rb2 && rb1->poseEstimate.isValid() &&
            rb2->poseEstimate.isValid() &&
            tag1->poseEstimate.isValid() && tag2->poseEstimate.isValid()) {
          used = tagGraph_.addDistanceMeasurement(rb1, rb2, tag1, tag2, *dm);
        }
      }
      if (!used) {
        dmv.push_back(dm);
      }
    }
    unappliedDistanceMeasurements_ = dmv;
  }
  
  void TagSlam::applyPositionMeasurements() {
    if (unappliedPositionMeasurements_.empty()) {
      return;
    }
    PositionMeasurementVec mv;
    for (const auto &m: unappliedPositionMeasurements_) {
      bool used(false);
      if (allTags_.count(m->tag) > 0) {
        const auto tag = allTags_[m->tag];
        const auto rb = findBodyForTag(m->tag, tag->bits);
        if (rb && rb->poseEstimate.isValid()) {
          used = tagGraph_.addPositionMeasurement(rb, tag, *m);
        }
      }
      if (!used) {
        mv.push_back(m);
      }
    }
    unappliedPositionMeasurements_ = mv;
  }

  static tf::Transform gtsam_pose_to_tf(const gtsam::Pose3 &p) {
    tf::Transform tf;
    const gtsam::Vector rpy = p.rotation().rpy();
    tf.setOrigin(tf::Vector3(p.x(), p.y(), p.z()));
    tf::Quaternion q;
    q.setRPY(rpy(0), rpy(1), rpy(2));
    tf.setRotation(q);
    return (tf);
  }

  static nav_msgs::Odometry
  make_odom(const ros::Time &t,
            const std::string &fixed_frame,
            const std::string &child_frame,
            const gtsam::Pose3 &pose) {
    nav_msgs::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = fixed_frame;
    odom.child_frame_id = child_frame;
    const auto TF = pose.matrix();
    Eigen::Affine3d TFa(TF);
    tf::poseEigenToMsg(TFa, odom.pose.pose);
    return (odom);
  }

  void TagSlam::broadcastCameraPoses(const ros::Time &t) {
    std::vector<PoseInfo> camPoseInfo;
    for (const auto cam_idx: irange(0ul, cameras_.size())) {
      const auto &cam = cameras_[cam_idx];
      PoseEstimate pe = tagGraph_.getCameraPose(cam, frameNum_);
      if (pe.isValid()) {
        const std::string frame_id = "cam_" + std::to_string(cam->index);
        camPoseInfo.push_back(PoseInfo(pe, t, frame_id));
        camOdomPub_[cam_idx].publish(make_odom(t, fixedFrame_,
                                               frame_id, pe.getPose()));
      }
    }
    broadcastTransforms(fixedFrame_, camPoseInfo);
  }

  
  void TagSlam::broadcastBodyPoses(const ros::Time &t) {
    std::vector<PoseInfo> bodyPoseInfo;
    for (const auto &rb: allBodies_) {
      const PoseEstimate &pe = rb->poseEstimate;
      if (pe.isValid()) {
        const std::string frame_id = "body_" + rb->name;
        bodyPoseInfo.push_back(PoseInfo(pe, t, frame_id));
      }
    }
    broadcastTransforms(fixedFrame_, bodyPoseInfo);
    // publish odom for dynamic bodies
    for (const auto body_idx : irange(0ul, dynamicBodies_.size())) {
      const auto rb = dynamicBodies_[body_idx];
      const PoseEstimate &pe = rb->poseEstimate;
      if (pe.isValid()) {
        const std::string frame_id = "body_" + rb->name;
        bodyOdomPub_[body_idx].publish(
          make_odom(t, fixedFrame_, frame_id, pe.getPose()));
      }
    }
  }

  void TagSlam::broadcastTagPoses(const ros::Time &t) {
    for (const auto &rb: allBodies_) {
      if (rb->poseEstimate.isValid()) {
        std::vector<PoseInfo> tagPoseInfo;
        for (auto &tg: rb->tags) {
          TagPtr tag = tg.second;
          if (tag->poseEstimate.isValid()) {
            const std::string frame_id = "tag_" + std::to_string(tag->id);
            tagPoseInfo.push_back(PoseInfo(tag->poseEstimate, t, frame_id));
          }
        }
        broadcastTransforms("body_" + rb->name, tagPoseInfo);
      }
    }
  }

  void TagSlam::broadcastTransforms(const std::string &parentFrame,
                                    const std::vector<PoseInfo> &poses) {
    for (const auto &p: poses) {
      const auto &tf = gtsam_pose_to_tf(p.pose);
      tfBroadcaster_.sendTransform(tf::StampedTransform(tf, p.time,
                                                        parentFrame, p.frame_id));
    }
  }

  
  void TagSlam::writeBodyPoses(const std::string &poseFile) const {
    std::ofstream pf(poseFile);
    pf << "bodies:" << std::endl;
    for (const auto &rb : allBodies_) {
      rb->write(pf, " ");
    }
  }

  void TagSlam::writeTagWorldPoses(const std::string &poseFile, unsigned int frameNum) const {
    std::ofstream pf(poseFile);
    for (const auto &rb : allBodies_) {
      for (const auto &tm: rb->tags) {
        const auto &tag = tm.second;
        const PoseEstimate pe = tagGraph_.getTagWorldPose(rb, tag->id, frameNum);
        if (pe.isValid()) {
          pf << "- id: "   << tag->id << std::endl;
          pf << "  size: " << tag->size << std::endl;
          yaml_utils::write_pose(pf, "  ", pe.getPose(), pe.getNoise(), true);
        }
      }
    }
  }

  
  void TagSlam::writeMeasurements(const std::string &fname,
                                  const PointVector &dist,
                                  const PointVector &pos) const {
    std::ofstream f(fname);
    writeDistanceMeasurements(f, dist);
    writePositionMeasurements(f, pos);
  }


  TagSlam::PointVector TagSlam::getDistances() const {
    PointVector pv;
    for (const auto &dm: distanceMeasurements_) {
      std::pair<gtsam::Point3, bool> p(gtsam::Point3(), false);
      if (allTags_.count(dm->tag1) > 0 &&
          allTags_.count(dm->tag2) > 0) {
        const auto tag1 = allTags_.find(dm->tag1)->second;
        const auto tag2 = allTags_.find(dm->tag2)->second;
        const auto rb1 = findBodyForTag(dm->tag1, tag1->bits);
        const auto rb2 = findBodyForTag(dm->tag2, tag2->bits);
        if (rb1 && rb2) {
          p = tagGraph_.getDifference(rb1, rb2, tag1, dm->corner1,
                                      tag2, dm->corner2);
        }
      }
      pv.push_back(p);
    }
    return (pv);
  }

  void TagSlam::writeDistanceMeasurements(std::ostream &f, const PointVector &dist) const {
    f << "distance_measurements:" << std::endl;
    for (const auto i: irange(0ul, distanceMeasurements_.size())) {
      const auto &dm = distanceMeasurements_[i];
      const auto &d  = dist[i];
      if (d.second) {
        double len = d.first.norm();
        f << "  - " << dm->name << ":" << std::endl;
        f << "      tag1: " << dm->tag1 << std::endl;
        f << "      tag2: " << dm->tag2 << std::endl;
        f << "      corner1: " << dm->corner1 << std::endl;
        f << "      corner2: " << dm->corner2 << std::endl;
        f << "      distance: " << dm->distance << std::endl;
        f << "      optimized: " << len << std::endl;
        f << "      error: " << len - dm->distance << std::endl;
        f << "      noise:  " << dm->noise << std::endl;
      }
    }
  }

  static std::pair<int, int> find_min_max_diff(const std::vector<double> &meas,
                                               const std::vector<double> &opt) {
    double minError(1e10), maxError(0);
    std::pair<int, int> minMaxIdx(-1, -1);
    for (const auto i: irange(0ul, meas.size())) {
      if (meas[i] > -1e10 && opt[i] > -1e10) {
        double err = std::abs(meas[i] - opt[i]);
        if (err < minError) {
          minError = err;
          minMaxIdx.first = i;
        }
        if (err > maxError) {
          maxError = err;
          minMaxIdx.second = i;
        }
      }
    }
    return (minMaxIdx);
  }

  static void print_error(const std::string &name, const DistanceMeasurementConstPtr &dm,
                          double meas) {
    const double err     = dm->distance - meas;
    ROS_INFO_STREAM(name << " err: " << dm->name << " meas: " << dm->distance << " optim: " << meas <<
                    " error: " << err << " noise: " << dm->noise);
  }

  void TagSlam::printDistanceErrors(const PointVector &dist) const {
    std::vector<double> dmeas, dopt;
    for (const auto i: irange(0ul, distanceMeasurements_.size())) {
      dmeas.push_back(distanceMeasurements_[i]->distance);
      dopt.push_back(dist[i].second ? dist[i].first.norm() : -1e10);
    }
    std::pair<int,int> minMax = find_min_max_diff(dmeas, dopt);
    if (minMax.first > 0) {
      print_error("minimum", distanceMeasurements_[minMax.first], dopt[minMax.first]);
      print_error("maximum", distanceMeasurements_[minMax.second], dopt[minMax.second]);
    } 
  }

  TagSlam::PointVector TagSlam::getPositions() const {
    PointVector pv;
    for (const auto &m: positionMeasurements_) {
      std::pair<gtsam::Point3, bool> p(gtsam::Point3(), false);
      if (allTags_.count(m->tag) > 0) {
        const auto tag = allTags_.find(m->tag)->second;
        const auto rb = findBodyForTag(m->tag, tag->bits);
        if (rb) {
          p = tagGraph_.getPosition(rb, tag, m->corner);
        }
      }
      pv.push_back(p);
    }
    return (pv);
  }

  static void print_error(const std::string &name, const PositionMeasurementConstPtr &pm,
                          double meas) {
    const double err     = pm->length - meas;
    ROS_INFO_STREAM(name << " err: " << pm->name << " meas: " << pm->length << " optim: " << meas <<
                    " error: " << err << " noise: " << pm->noise);
  }

  void TagSlam::printPositionErrors(const PointVector &pos) const {
    std::vector<double> pmeas, popt;
    for (const auto i: irange(0ul, positionMeasurements_.size())) {
      const auto &pm = positionMeasurements_[i];
      pmeas.push_back(pm->length);
      popt.push_back(pos[i].second ? pos[i].first.dot(pm->dir) : -1e10);
    }
    std::pair<int,int> minMax = find_min_max_diff(pmeas, popt);
    if (minMax.first > 0) {
      print_error("minimum", positionMeasurements_[minMax.first], popt[minMax.first]);
      print_error("maximum", positionMeasurements_[minMax.second], popt[minMax.second]);
    } 
  }


  void TagSlam::writePositionMeasurements(std::ostream &f, const PointVector &pos) const {
    f << "position_measurements:" << std::endl;
    for (const auto i: irange(0ul, positionMeasurements_.size())) {
      const auto &m = positionMeasurements_[i];
      const auto &p = pos[i];
      if (p.second) {
        double len = p.first.dot(m->dir);
        f << "  - " << m->name << ":" << std::endl;
        f << "      tag: " << m->tag << std::endl;
        f << "      corner: " << m->corner << std::endl;
        f << "      length: " << m->length << std::endl;
        f << "      measured: " << len << std::endl;
        f << "      error: " << len - m->length << std::endl;
        f << "      noise: " << m->noise << std::endl;
        f << "      direction: [ " << m->dir.x() << ", " << m->dir.y() << ", " << m->dir.z() << "]" << std::endl;
      }
    }
  }

  void TagSlam::callback1(TagArrayConstPtr const &tag0) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0};
    processTags(msg_vec);
  }

  void TagSlam::callback2(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1};
    processTags(msg_vec);
  }
  void TagSlam::callback3(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1,
                          TagArrayConstPtr const &tag2) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1, tag2};
    processTags(msg_vec);
  }
  void TagSlam::callback4(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1,
                          TagArrayConstPtr const &tag2,
                          TagArrayConstPtr const &tag3) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1, tag2, tag3};
    processTags(msg_vec);
  }
  void TagSlam::callback5(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1,
                          TagArrayConstPtr const &tag2,
                          TagArrayConstPtr const &tag3,
                          TagArrayConstPtr const &tag4) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1, tag2, tag3, tag4};
    processTags(msg_vec);
  }
  void TagSlam::callback6(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1,
                          TagArrayConstPtr const &tag2,
                          TagArrayConstPtr const &tag3,
                          TagArrayConstPtr const &tag4,
                          TagArrayConstPtr const &tag5) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1, tag2, tag3, tag4, tag5};
    processTags(msg_vec);
  }
  void TagSlam::callback7(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1,
                          TagArrayConstPtr const &tag2,
                          TagArrayConstPtr const &tag3,
                          TagArrayConstPtr const &tag4,
                          TagArrayConstPtr const &tag5,
                          TagArrayConstPtr const &tag6) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1, tag2, tag3, tag4, tag5, tag6};
    processTags(msg_vec);
  }
  void TagSlam::callback8(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1,
                          TagArrayConstPtr const &tag2,
                          TagArrayConstPtr const &tag3,
                          TagArrayConstPtr const &tag4,
                          TagArrayConstPtr const &tag5,
                          TagArrayConstPtr const &tag6,
                          TagArrayConstPtr const &tag7) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1, tag2, tag3, tag4, tag5, tag6, tag7};
    processTags(msg_vec);
  }

  void TagSlam::processImages(const std::vector<ImageConstPtr> &msgvec) {
    images_.clear();
    for (const auto i: irange(0ul, msgvec.size())) {
      const auto &img = msgvec[i];
      cv::Mat im = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8)->image;
      images_.push_back(im);
    }
  }
  void TagSlam::processTagsAndImages(const std::vector<TagArrayConstPtr> &msgvec1,
                                     const std::vector<ImageConstPtr> &msgvec2) {
    processImages(msgvec2);
    processTags(msgvec1);
  }

  void TagSlam::finalize() {
    unsigned int frameNum = (unsigned int)std::max(0, (int)frameNum_ - 1);
    tagGraph_.computeMarginals();
    updatePosesFromGraph(frameNum);
    writeBodyPoses(tagPosesOutFile_);
    writeTagWorldPoses(tagWorldPosesOutFile_, frameNum);
    writeMeasurements(measurementsOutFile_, getDistances(), getPositions());
  }


  void TagSlam::playFromBag(const std::string &fname) {
    rosbag::Bag bag;
    bag.open(fname, rosbag::bagmode::Read);
    std::vector<std::string> tagTopics, imageTopics, topics;
    for (const auto cam_idx: irange(0ul, cameras_.size())) {
      const auto &cam = cameras_[cam_idx];
      tagTopics.push_back(cam->tagtopic);
      imageTopics.push_back(cam->rostopic);
      topics.push_back(imageTopics.back());
      topics.push_back(tagTopics.back());
    }
    ROS_INFO_STREAM("playing from file: " << fname);
    double start_time(0);
    nh_.param<double>("start_time", start_time, 0);
    ros::Time t_start(start_time);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View t0View(bag);
    ros::Time t0(0.0);
    for (const rosbag::MessageInstance &m: t0View) {
      t0 = m.getTime();
      break;
    }
    ROS_INFO_STREAM("start bag time: " << t0);

    BagSync2<TagArray, Image> tagImageSync(tagTopics, imageTopics,
                                           std::bind(&TagSlam::processTagsAndImages, this,
                                                     std::placeholders::_1, std::placeholders::_2));
    //BagSync<TagArray> tagSync(tagTopics, std::bind(&TagSlam::processTags, this, std::placeholders::_1));

    //BagSync<TagArray> tagSync(tagTopics,  std::bind(&SyncMerge::processSync1, &syncMerge, std::placeholders::_1));
    //BagSync<Image> imageSync(imageTopics, std::bind(&SyncMerge::processSync2, &syncMerge, std::placeholders::_1));

    std::ofstream wand_poses("wand_poses.txt");
    ros::Time lastTime(0.0);
    for (const rosbag::MessageInstance &m: view) {
      tagImageSync.process(m);
      const auto t = tagImageSync.getCurrentTime();
      if (!dynamicBodies_.empty() && dynamicBodies_[0]->poseEstimate.isValid()
          && t != lastTime) {
        lastTime = t;
        const PoseEstimate &pe = dynamicBodies_[0]->poseEstimate;
        const gtsam::Point3 tip = (pe * allTags_[4]->poseEstimate).translation();
        const gtsam::Vector rv = gtsam::Rot3::Logmap(pe.rotation());
        const gtsam::Point3 T = pe.translation();
        wand_poses << t-t0 << " " << t << " " << rv(0) << " " << rv(1) << " " << rv(2) << " "
                   << T.x()   << " " << T.y()   << " " << T.z() << " "
                   << tip.x() << " " << tip.y() << " " << tip.z() << std::endl;
        wand_poses << std::flush;
      }
      if (frameNum_ > (unsigned int)maxFrameNum_) {
        break;
      }
    }
    bag.close();
    finalize();
  }
  
}  // namespace
