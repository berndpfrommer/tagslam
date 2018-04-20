/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_slam.h"
#include "tagslam/pose_estimate.h"
#include "tagslam/tag.h"
#include "tagslam/yaml_utils.h"
#include "tagslam/rigid_body.h"
#include <XmlRpcException.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/range/irange.hpp>
#include <math.h>
#include <fstream>
#include <iomanip>

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

    readDistanceMeasurements();
    if (!readRigidBodies()) {
      return (false);
    }
    for (const auto &rb: dynamicBodies_) {
      bodyOdomPub_.push_back(
        nh_.advertise<nav_msgs::Odometry>("odom/body_" + rb->name, 1));
    }
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
  
  void TagSlam::readDistanceMeasurements() {
    // read distance measurements
    XmlRpc::XmlRpcValue distMeas;
    nh_.getParam("tag_poses/distance_measurements", distMeas);
    if (distMeas.getType() == XmlRpc::XmlRpcValue::TypeArray) { 
      distanceMeasurements_ = DistanceMeasurement::parse(distMeas);
      ROS_INFO_STREAM("found " << distanceMeasurements_.size()
                      << " distance measurements!");
    } else {
      ROS_INFO("no distance measurements found!");
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
    if (bodies.getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
      ROS_ERROR("cannot find bodies in yaml file!");
      return (false);
    }
    RigidBodyVec rbv = RigidBody::parse_bodies(body_defaults, bodies);
    ROS_INFO_STREAM("configured bodies: " << rbv.size());
    if (rbv.size() > tagGraph_.getMaxNumBodies()) {
      ROS_ERROR_STREAM("too many bodies, max is: "
                       << tagGraph_.getMaxNumBodies());
    }
    if (rbv.empty()) {
      rbv.emplace_back(new RigidBody("world", true/*isStatic*/));
      PoseEstimate pe(gtsam::Pose3(), 0.0/*err*/, 0);
      rbv.back()->setPoseEstimate(pe);
      rbv.back()->setIsDefaultBody(true);
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
      tagGraph_.addDistanceMeasurements(distanceMeasurements_);
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
      gtsam::Pose3 p;
      if (tagGraph_.getBodyPose(rb, &p, frame)) {
        std::cout << "UPDATE: body " << rb->name << " from " << std::endl;
        std::cout << rb->poseEstimate.getPose() << std::endl << " to: " << std::endl << p << std::endl;
        rb->poseEstimate = PoseEstimate(p, 0.0, 0);
      } else {
        if (!rb->isStatic) {
          rb->poseEstimate = PoseEstimate();//invalid
        }
      }
      if (rb->poseEstimate.isValid()) {
        for (auto &t: rb->tags) {
          TagPtr tag = t.second;
          gtsam::Pose3 pose;
          if (tagGraph_.getTagRelPose(rb, tag->id, &pose)) {
            std::cout << "UPDATE: tag id " << tag->id << " from: " << std::endl <<
              tag->poseEstimate.getPose()  << std::endl << " to: " << std::endl <<  pose << std::endl;
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

#define DEBUG_POSE_ESTIMATE
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
#if 0    
    PoseEstimate pe = pointsArePlanar? estimatePoseHomography(cam_idx, wp, ip) :
      estimatePosePNP(cam_idx, wp, ip);
#else
    PoseEstimate pe = estimatePosePNP(cam_idx, wp, ip);
#endif    

    std::cout << "pnp pose estimate has error: " << pe.getError() << std::endl;
    if (!pe.isValid()) {
#if 0      
      // PNP failed, try with a local graph
      const PoseEstimate &prevPose = pointsArePlanar ? pe :
        cameras_[cam_idx]->poseEstimate;
#else      
      // PNP failed, try with a local graph
      const PoseEstimate &prevPose = cameras_[cam_idx]->poseEstimate;
      std::cout << "running mini graph for cam " << cam_idx << " prev pose: " << std::endl << prevPose << std::endl;
#endif      
      pe = initialPoseGraph_.estimateCameraPose(cameras_[cam_idx], wp, ip,
                                                prevPose);
      std::cout << "mini graph estimate err: " << pe.getError() << std::endl;
      if (pe.getError() > 100.0) {
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
      std::cout << "initial cam pose: " << cameras_[cam_idx]->poseEstimate << std::endl;
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
        std::cout << "IGNORING tag " << tag->id << " (bad viewing angle)" << std::endl;
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
      std::cout << "init tag pose est: " << tag->id << std::endl << " T_c_o: " << pe.inverse() << std::endl;
      std::cout << "T_w_c: " << cam->poseEstimate.getPose() << std::endl;
      std::cout << "T_b_w: " << bodyPose.inverse() <<  std::endl;
      std::cout << "T_b_o: " << tag->poseEstimate.getPose() <<  std::endl;
    }
    std::cout << "pose estimate for tag " << tag->id << ": " << pe << std::endl;
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
                if (estimateTagPose(tagMap.first, rb->poseEstimate.getPose(), tag)) {
                  TagVec tvec = {tag};
                  tagGraph_.addTags(rb, tvec);
                  tagGraph_.addDistanceMeasurements(distanceMeasurements_);
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
        
  void TagSlam::process(const std::vector<TagArrayConstPtr> &msgvec) {
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
    // For dynamic bodies, find their initial poses if
    // any of their tags are observed
    findInitialBodyPoses();
    // Any newly discovered tags can now be given
    // an initial pose, too.
    findInitialDiscoveredTagPoses();
    
    runOptimizer();
    updatePosesFromGraph(frameNum_);
    computeProjectionError();
    detachObservedTagsFromBodies();

    ros::Time t = get_latest_time(msgvec);
    broadcastCameraPoses(t);
    broadcastBodyPoses(t);
    broadcastTagPoses(t);
    writeTagPoses(tagPosesOutFile_);
    writeTagWorldPoses(tagWorldPosesOutFile_);
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
    std::cout << "&&&& estimating body pose for " << rb->name << std::endl;
    PoseEstimate bodyPose; // defaults to invalid pose estimate
    int best_cam_idx = rb->bestCamera();
    /*
    for (const auto cam_idx: irange(0ul, cameras_.size())) {
      RigidBodyConstVec rbv = {rb};
      PoseEstimate pe  = findCameraPose(cam_idx, rbv, false);
      if (pe.isValid()) {
        // pose estimate has T_b_c
        // T_w_b = T_w_c * T_c_b
        const auto &T_w_c = cameras_[cam_idx]->poseEstimate;
        PoseEstimate bPose = PoseEstimate(T_w_c * pe.inverse(), 0.0, 0);
        std::cout << "------- body pose from camera " << cam_idx << std::endl;
        std::cout << "T_w_c: " << std::endl << T_w_c << std::endl;
        std::cout << "T_c_b: " << std::endl << pe.inverse() << std::endl;
        std::cout << "body pose: " << std::endl <<  bPose << std::endl;
        if (cam_idx == best_cam_idx) {
          bodyPose = bPose;
        }
        for (const auto cam_idx2: irange(0ul, cameras_.size())) {
          RigidBodyPtr rb2(new RigidBody(*rb));
          rb2->poseEstimate = bPose;
          RigidBodyConstVec rbv2{rb2};
          PoseEstimate cpe = findCameraPose(cam_idx2, rbv2, true);
          if (cpe.isValid()) {
            std::cout << "camera: " << cam_idx2 << " has derived world pose: " << std::endl
                      << cpe.getPose() << std::endl;
          }
        }
      }
    }
    */
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
    std::cout << "body pose from single camera " << best_cam_idx << std::endl;
    std::cout << bPose << std::endl;

    bodyPose = initialPoseGraph_.estimateBodyPose(cameras_, rb, bPose);
    std::cout << "body pose from body graph: " << std::endl << bodyPose << std::endl;
#if 0    
    int cam_idx = rb->bestCamera();
    if (cam_idx >= 0) {
      std::cout << "USING cam " << cam_idx << " to find best pose of body: " << rb->name << std::endl;
      RigidBodyConstVec rbv = {rb};
      PoseEstimate pe  = findCameraPose(cam_idx, rbv, false);
      if (pe.isValid()) {
        // pose estimate has T_b_c
        // T_w_b = T_w_c * T_c_b
        const auto &T_w_c = cameras_[cam_idx]->poseEstimate;
        bodyPose = PoseEstimate(T_w_c * pe.inverse(), 0.0, 0);
        std::cout << "T_w_c: " << T_w_c << std::endl;
        std::cout << "T_c_b: " << pe.inverse() << std::endl;
        std::cout << "body pose: " << pe.inverse() << std::endl;
      }
    }
#endif    
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
          // We encountered tags on a static body for the first time.
          // Any of these tags that have a known pose estimate can
          // be added to the graph and the global set of known tags.
          TagVec tvec;
          for (auto &t: rb->tags) {
            if (t.second->poseEstimate.isValid()) {
              tvec.push_back(t.second);
              allTags_.insert(t);
            }
          }
          tagGraph_.addTags(rb, tvec);
          tagGraph_.addDistanceMeasurements(distanceMeasurements_);
        }
      }
    }
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

  
  static void write_vec(std::ofstream &of,
                        const std::string &prefix,
                        double x, double y, double z) {
    const int p(8);
    of.precision(p);
    of << prefix << "x: " << std::fixed << x << std::endl;
    of << prefix << "y: " << std::fixed << y << std::endl;
    of << prefix << "z: " << std::fixed << z << std::endl;
  }

  static void write_pose(std::ofstream &of, const std::string &prefix,
                         const gtsam::Pose3 &pose,
                         const PoseNoise &n) {
    gtsam::Vector r = gtsam::Rot3::Logmap(pose.rotation());
    gtsam::Vector t(pose.translation());
    const std::string pps = prefix + "  ";
    of << prefix << "center:" << std::endl;
    write_vec(of, pps, t(0), t(1), t(2));
    of << prefix << "rotvec:" << std::endl;
    write_vec(of, pps, r(0), r(1), r(2));
 
    gtsam::Vector nvec = n->sigmas();
    of << prefix << "position_noise:" << std::endl;
    write_vec(of, pps, nvec(3),nvec(4),nvec(5));
    of << prefix << "rotation_noise:" << std::endl;
    write_vec(of, pps, nvec(0),nvec(1),nvec(2));
  }

  void TagSlam::writeTagPoses(const std::string &poseFile) const {
    std::ofstream pf(poseFile);
    pf << "bodies:" << std::endl;
    PoseNoise smallNoise = makePoseNoise(0.001, 0.001);
    for (const auto &rb : allBodies_) {
      pf << " - " << rb->name << ":" << std::endl;
      std::string pfix = "     ";
      pf << pfix << "is_default_body: " <<
        (rb->isDefaultBody ? "true" : "false") << std::endl;
      pf << pfix << "is_static: " <<
        (rb->isStatic ? "true" : "false") << std::endl;
      pf << pfix << "default_tag_size: " << rb->defaultTagSize << std::endl;
      if (rb->isStatic) {
        pf << pfix << "pose:" << std::endl;;
        write_pose(pf, pfix + "  ", rb->poseEstimate, smallNoise);
        //rb->poseEstimate.getNoise());
      }
      pf << pfix << "tags: " << std::endl;
      for (const auto &tm: rb->tags) {
        const auto &tag = tm.second;
        pf << pfix << "- id: "   << tag->id << std::endl;
        pf << pfix << "  size: " << tag->size << std::endl;
        if (tag->poseEstimate.isValid()) {
          write_pose(pf, pfix + "  ", tag->poseEstimate, smallNoise);
          //tag->poseEstimate.getNoise());
        }
      }
    }
  }
  void TagSlam::writeTagWorldPoses(const std::string &poseFile) const {
    std::ofstream pf(poseFile);
    for (const auto &rb : allBodies_) {
      for (const auto &tm: rb->tags) {
        const auto &tag = tm.second;
        const PoseEstimate pe = tagGraph_.getTagWorldPose(rb, tag->id, frameNum_);
        if (pe.isValid()) {
          pf << "- id: "   << tag->id << std::endl;
          pf << "  size: " << tag->size << std::endl;
          write_pose(pf, "  ", pe.getPose(), pe.getNoise());
        }
      }
    }
  }

  void TagSlam::callback1(TagArrayConstPtr const &tag0) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0};
    process(msg_vec);
  }

  void TagSlam::callback2(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1};
    process(msg_vec);
  }
  void TagSlam::callback3(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1,
                          TagArrayConstPtr const &tag2) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1, tag2};
    process(msg_vec);
  }
  void TagSlam::callback4(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1,
                          TagArrayConstPtr const &tag2,
                          TagArrayConstPtr const &tag3) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1, tag2, tag3};
    process(msg_vec);
  }
  void TagSlam::callback5(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1,
                          TagArrayConstPtr const &tag2,
                          TagArrayConstPtr const &tag3,
                          TagArrayConstPtr const &tag4) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1, tag2, tag3, tag4};
    process(msg_vec);
  }
  void TagSlam::callback6(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1,
                          TagArrayConstPtr const &tag2,
                          TagArrayConstPtr const &tag3,
                          TagArrayConstPtr const &tag4,
                          TagArrayConstPtr const &tag5) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1, tag2, tag3, tag4, tag5};
    process(msg_vec);
  }
  void TagSlam::callback7(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1,
                          TagArrayConstPtr const &tag2,
                          TagArrayConstPtr const &tag3,
                          TagArrayConstPtr const &tag4,
                          TagArrayConstPtr const &tag5,
                          TagArrayConstPtr const &tag6) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1, tag2, tag3, tag4, tag5, tag6};
    process(msg_vec);
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
    process(msg_vec);
  }


  void TagSlam::playFromBag(const std::string &fname) {
    rosbag::Bag bag;
    bag.open(fname, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    std::map<std::string, int> topic_to_cam;
    for (const auto cam_idx: irange(0ul, cameras_.size())) {
      const auto &cam = cameras_[cam_idx];
      topics.push_back(cam->tagtopic);
      topic_to_cam.insert(std::map<std::string, int>::value_type(cam->tagtopic, cam_idx));
    }
    ROS_INFO_STREAM("playing from file: " << fname);
    double start_time(0);
    nh_.param<double>("start_time", start_time, 0);
    ros::Time t_start(start_time);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    std::map<std::string, boost::shared_ptr<TagArray>> msg_map;
    ros::Time currentTime(0.0);
    for (const rosbag::MessageInstance &m: view) {
      boost::shared_ptr<TagArray> tags = m.instantiate<TagArray>();
      if (tags) {
        if (tags->header.stamp > currentTime) {
          if (msg_map.size() == topics.size()) {
            std::vector<TagArrayConstPtr> msg_vec;
            for (const auto &m: msg_map) {
              msg_vec.push_back(m.second);
            }
            process(msg_vec);
            //break; // XXX
          }
          msg_map.clear();
          currentTime = tags->header.stamp;
        }
        msg_map[m.getTopic()] = tags;
      }
    }
    bag.close();
  }

}  // namespace
