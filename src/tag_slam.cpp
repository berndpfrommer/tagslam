/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_slam.h"
#include "tagslam/pose_estimate.h"
#include "tagslam/tag.h"
#include "tagslam/yaml_utils.h"
#include "tagslam/rigid_body.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
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
    XmlRpc::XmlRpcValue bodies;
    nh_.getParam("tag_poses/bodies", bodies);
    nh_.param<bool>("discover_tags", discoverTags_, true);
    nh_.param<std::string>("tag_poses_out_file", tagPosesOutFile_,
                           "poses_out.yaml");
    if (bodies.getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
      ROS_ERROR("cannot find bodies in yaml file!");
      return (false);
    }
    RigidBodyVec rbv = RigidBody::parse_bodies(bodies);
    ROS_INFO_STREAM("configured bodies: " << rbv.size());
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
        tvec.push_back(t.second);
        allTags_.insert(t);
      }
      tagGraph_.addTags(rb, tvec);
      allBodies_.push_back(rb);
      (rb->isStatic ? staticBodies_ : dynamicBodies_).push_back(rb);
    }
    // play from bag file if file name is non-empty
    std::string bagFile;
    nh_.param<std::string>("bag_file", bagFile, "");
    if (!bagFile.empty()) {
      playFromBag(bagFile);
      ros::shutdown();
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
        approxSync3_.reset(new TimeSync3(SyncPolicy3(60/*q size*/),
                                         *(sub_[0]), *(sub_[1]), *(sub_[2])));
        approxSync3_->registerCallback(&TagSlam::callback3, this);
        break;
      default:
        ROS_ERROR_STREAM("number of cameras too large: " << cameras_.size());
        return (false);
        break;
      }
    }
    return (true);
  }

  static ros::Time get_latest_time(const std::vector<TagArrayConstPtr> &msgvec) {
   ros::Time t(0);
    for (const auto &m: msgvec) {
      if (m->header.stamp > t) t = m->header.stamp;
    }
    return (t);
  }

//#define DEBUG_CAM_POSE_INIT

  PoseEstimate
  TagSlam::estimatePosePNP(int cam_idx,
                           const std::vector<gtsam::Point3>&wpts,
                           const std::vector<gtsam::Point2>&ipts) const {
    PoseEstimate pe;
    if (!ipts.empty()) {
      std::vector<cv::Point3d> wp;
      std::vector<cv::Point2d> ip;
      
      for (const auto &w: wpts) {
        wp.emplace_back(w.x(), w.y(), w.z());
      }
      for (const auto &i: ipts) {
        ip.emplace_back(i.x(), i.y());
      }
      const auto   &ci  = cameras_[cam_idx]->intrinsics;
      cv::Mat rvec, tvec;
      bool rc = utils::get_init_pose_pnp(wp, ip, ci.K,
                                         ci.distortion_model,
                                         ci.D, &rvec, &tvec);
      if (rc) {
        gtsam::Vector tvec_gtsam = (gtsam::Vector(3) <<
                                    tvec.at<double>(0),
                                    tvec.at<double>(1),
                                    tvec.at<double>(2)).finished();
        gtsam::Pose3 T_c_w(gtsam::Rot3::rodriguez(
                                   rvec.at<double>(0),
                                   rvec.at<double>(1),
                                   rvec.at<double>(2)), tvec_gtsam);
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
      if (!rb->isStatic) {
        gtsam::Pose3 p;
        if (tagGraph_.getBodyPose(rb, &p, frame)) {
          rb->poseEstimate = PoseEstimate(p);
        } else {
          rb->poseEstimate = PoseEstimate();//invalid
        }
      }
      if (rb->poseEstimate.isValid()) {
        for (auto &t: rb->tags) {
          TagPtr tag = t.second;
          gtsam::Pose3 pose;
          if (tagGraph_.getTagRelPose(rb, tag->id, &pose)) {
            tag->poseEstimate = PoseEstimate(pose, 0.0, 0);
          } else {
            tag->poseEstimate = PoseEstimate(); // invalid
          }
        }
      }
    }
  }

  void TagSlam::attachObservedTagsToBodies(
    const std::vector<TagArrayConstPtr> &msgvec) {
    for (const auto cam_idx: irange(0ul, msgvec.size())) {
      const auto &tags = msgvec[cam_idx];
      for (auto &body : allBodies_) {
        body->attachObservedTags(cam_idx, tags);
      }
    }
  }

  RigidBodyPtr
  TagSlam::findBodyForTag(int tagId) const {
    for (auto &body : allBodies_) {
      if (body->hasTag(tagId)) return (body);
    }
    return (NULL);
  }

  void TagSlam::discoverTags(const std::vector<TagArrayConstPtr> &msgvec) {
    for (const auto cam_idx: irange(0ul, msgvec.size())) {
      const auto &tags = msgvec[cam_idx];
      for (const auto &t: tags->apriltags) {
        // found new tag
        if (allTags_.count(t.id) == 0) {
          RigidBodyPtr body = findBodyForTag(t.id);
          if (body) {
            TagPtr tag = body->addDefaultTag(t.id);
            ROS_INFO_STREAM("found new tag: " << tag->id
                            << " of size: " << tag->size
                            << " for body: " << body->name);
            allTags_.insert(IdToTagMap::value_type(tag->id, tag));
          }
        }
      }
    }
  }

  PoseEstimate
  TagSlam::poseFromPoints(int cam_idx,
                          const std::vector<gtsam::Point3> &wp,
                          const std::vector<gtsam::Point2> &ip) const {
    PoseEstimate pe = estimatePosePNP(cam_idx, wp, ip);
    if (pe.getError() > 10.0) {
      // PNP failed, try with a local graph
      const PoseEstimate &prevPose = cameras_[cam_idx]->poseEstimate;
      pe = initialPoseGraph_.estimateCameraPose(cameras_[cam_idx], wp, ip,
                                                prevPose);
      if (pe.getError() > 10.0) {
        ROS_WARN_STREAM("cannot find pose of camera " <<
                        cam_idx << " err: " << pe.getError());
        pe.setValid(false);
      }
    }
    return (pe);
  }

  PoseEstimate
  TagSlam::findCameraPose(int cam_idx, RigidBodyVec &rigidBodies) const {
    std::vector<gtsam::Point3> wp;
    std::vector<gtsam::Point2> ip;
    for (const auto &rb : rigidBodies) {
      rb->getAttachedPoints(cam_idx, &wp, &ip);
    }
    return (poseFromPoints(cam_idx, wp, ip));
  }
                                       

  void TagSlam::findInitialCameraPoses() {
    for (const auto cam_idx: irange(0ul, cameras_.size())) {
      cameras_[cam_idx]->poseEstimate =
        findCameraPose(cam_idx, staticBodies_);
    }
  }

#if 0  
  void TagSlam::findInitialBodyPoses() {
    for (auto &rb: dynamicBodies_) {
      int cam_idx = rb->cameraWithMostAttachedPoints();
      if (cam_idx < 0) {
        continue;
      }
      const CameraPtr &cam = cameras_[cam_idx];
      if (cam->poseEstimate.isValid()) {
        std::vector<gtsam::Point3> wp;
        std::vector<gtsam::Point2> ip;
        rb->getAttachedPoints(cam_idx, &wp, &ip);
        const auto pe = poseFromPoints(cam_idx, wp, ip);
        if (pe.isValid()) {
          const auto &T_c_b = pe.pose;
          // T_w_b = T_w_c * T_c_b;
          rb->pose = cam->poseEstimate.pose.inverse() * T_c_b;
          rb->hasValidPoseEstimate = true;
        }
      }
    }
  }
#endif
  
  void
  TagSlam::estimateTagPose(int cam_idx,
                           const gtsam::Pose3 &bodyPose,
                           const TagPtr &tag) const {
    const auto &wp = tag->getObjectCorners();
    const auto ip = tag->getImageCorners();
    PoseEstimate pe = poseFromPoints(cam_idx, wp, ip);
    if (pe.isValid()) {
      const CameraPtr &cam = cameras_[cam_idx];
      // pose estimate has T_c_o
      // camera pose has T_c_w
      // tag pose should have T_b_o
      // T_b_o = T_b_w * T_w_c * T_c_o
      gtsam::Pose3 pose = bodyPose.inverse() *
        cam->poseEstimate.inverse() * pe;
      tag->poseEstimate = PoseEstimate(pose, 0.0, 0);
    }
  }
                                
  void TagSlam::findInitialDiscoveredTagPoses() {
    for (const auto &rb: dynamicBodies_) {
      if (rb->poseEstimate.isValid()) {
        // body has valid pose, let's see what
        // observed tags it has
        for (const auto &tagMap: rb->observedTags) {
          const CameraPtr &cam = cameras_[tagMap.first];
          if (cam->poseEstimate.isValid()) {
            for (const auto &tag: tagMap.second) {
              if (!tag->poseEstimate.isValid()) {
                estimateTagPose(tagMap.first, rb->poseEstimate, tag);
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
    if (discoverTags_) {
      // check if any of the tags are new, and associate them
      // with a rigid body
      discoverTags(msgvec);
    }
    // Sort the tags according to which bodies they
    // belong to. The observed tags are then hanging
    // off of the bodies, to be used subsequently
    attachObservedTagsToBodies(msgvec);
    // Go over all bodies and use tags with
    // established positions to determine camera poses
    findInitialCameraPoses();
    // For dynamic bodies, find their initial poses if
    // any of their tags are observed
    findInitialBodyPoses();
    // Any newly discovered tags can now be given
    // an initial pose, too.
    if (discoverTags_) {
      findInitialDiscoveredTagPoses();
    }
    runOptimizer();
    detachObservedTagsFromBodies();
    updatePosesFromGraph(frameNum_);
    ros::Time t = get_latest_time(msgvec);
    broadcastCameraPoses(t);
    broadcastTagPoses(t);
    writeTagPoses(tagPosesOutFile_);
    frameNum_++;
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
  }

  void TagSlam::findInitialBodyPoses() {
    // For each dynamic body, make body-frame 3d points
    // from all tags that have a valid pose estimate

    for (auto &rb: dynamicBodies_) {
      int cam_idx = rb->cameraWithMostAttachedPoints();
      rb->poseEstimate = PoseEstimate(); // invalid
      if (cam_idx >= 0) {
        RigidBodyVec rbv = {rb};
        PoseEstimate pe  = findCameraPose(cam_idx, rbv);
        if (pe.isValid()) {
          // pose estimate has T_c_body
          // T_w_body = T_w_c * T_c_body
          const auto &T_c_w = cameras_[cam_idx]->poseEstimate;
          rb->poseEstimate = PoseEstimate(T_c_w.inverse() * pe, 0.0, 0);
        }
      }
    }
  }

#if 0
  void TagSlam::findInitialDiscoveredTagPoses() {
    for (auto &rb: allBodies_) {
      if (!rb.hasValidPoseEstimate) {
        continue;
      }
      for (const auto cam_idx: irange(0ul, cameras_.size())) {
        if (!cameras_[cam_idx].hasValidPoseEstimate) {
          continue;
        }
        const auto discoveredTags = rb.getDiscoveredTags(cam_idx);
        for (auto &tag: discoveredTags) {
          if (!tag->hasPoseEstimate) {
            // get world and image points from this tags
            const auto &wp = tag->getObjectCorners();
            const auto ip  = tag->getImageCorners();
            PoseEstimate pe = poseFromPoints(cam_idx, wp, ip);
            if (pe.err < 1e10) {
              // pose estimate has T_c_o
              const auto &T_c_o = pe.pose;
              const auto &T_w_c = cameras_[cam_idx].pose;
              // T_w_o = T_w_c * T_c_o;
              // T_w_o = T_w_b * T_b_o;
              // --> T_b_o = T_b_w * T_w_c * T_c_o
              tag->pose = rb.pose.inverse() * T_w_c * T_c_o;
              tag->hasPoseEstimate = true;
            }
          }
        }
      }
    }
  }
#endif

  static tf::Transform gtsam_pose_to_tf(const gtsam::Pose3 &p) {
    tf::Transform tf;
    const gtsam::Vector rpy = p.rotation().rpy();
    tf.setOrigin(tf::Vector3(p.x(), p.y(), p.z()));
    tf::Quaternion q;
    q.setRPY(rpy(0), rpy(1), rpy(2));
    tf.setRotation(q);
    return (tf);
  }


  void TagSlam::broadcastCameraPoses(const ros::Time &t) {
    std::vector<PoseInfo> camPoseInfo;
    for (const auto &cam: cameras_) {
      PoseEstimate pe = tagGraph_.getCameraPose(cam, frameNum_);
      if (pe.isValid()) {
        const std::string frame_id = "cam_" + std::to_string(cam->index);
        camPoseInfo.push_back(PoseInfo(pe, t, frame_id));
      }
    }
    broadcastTransforms(camPoseInfo);
  }

  void TagSlam::broadcastTagPoses(const ros::Time &t) {
    std::vector<std::pair<int, gtsam::Pose3>> tagPoses;
    tagGraph_.getTagWorldPoses(&tagPoses);
    std::vector<PoseInfo> tagPoseInfo;
    for (const auto &cp: tagPoses) {
      const std::string frame_id = "tag_" + std::to_string(cp.first);
      tagPoseInfo.push_back(PoseInfo(cp.second, t, frame_id));
    }
    broadcastTransforms(tagPoseInfo);
  }

  void TagSlam::broadcastTransforms(const std::vector<PoseInfo> &poses) {
    for (const auto &p: poses) {
      const auto &tf = gtsam_pose_to_tf(p.pose);
      tfBroadcaster_.sendTransform(tf::StampedTransform(tf, p.time, "world", p.frame_id));
    }
  }

  
  static void write_vec(std::ofstream &of,
                        const std::string &prefix,
                        double x, double y, double z) {
    const int p(5);
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
    for (const auto &rb : allBodies_) {
      pf << " - " << rb->name << ":" << std::endl;
      std::string pfix = "     ";
      pf << pfix << "is_default_body: " <<
        (rb->isDefaultBody ? "true" : "false") << std::endl;
      pf << pfix << "default_size: " << rb->defaultSize << std::endl;
      if (rb->isStatic) {
        pf << pfix << "pose:" << std::endl;;
        write_pose(pf, pfix + "  ", rb->poseEstimate,
                   rb->poseEstimate.getNoise());
      }
      pf << pfix << "tags: " << std::endl;
      for (const auto &tm: rb->tags) {
        const auto &tag = tm.second;
        pf << pfix << "- id: "   << tag->id << std::endl;
        pf << pfix << "  size: " << tag->size << std::endl;
        if (tag->poseEstimate.isValid()) {
          write_pose(pf, pfix + "  ", tag->poseEstimate,
                     tag->poseEstimate.getNoise());
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


  void TagSlam::playFromBag(const std::string &fname) {
    if (cameras_.size() != 1) {
      ROS_ERROR("bag playback is supported only for single camera!");
      ros::shutdown();
    }
    rosbag::Bag bag;
    bag.open(fname, rosbag::bagmode::Read);
    std::vector<std::string> topics = {cameras_[0]->tagtopic};
    ROS_INFO_STREAM("playing from file: " << fname << " topic: " << topics[0]);
    double start_time(0);
    nh_.param<double>("start_time", start_time, 0);
    ros::Time t_start(start_time);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (const rosbag::MessageInstance &m: view) {
      boost::shared_ptr<TagArray> tags = m.instantiate<TagArray>();
      if (tags) {
        if (tags->header.stamp > t_start) {
          callback1(tags);
        }
      }
    }
    bag.close();
  }

}  // namespace
