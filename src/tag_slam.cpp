/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_slam.h"
#include <boost/range/irange.hpp>
#include "tagslam/tag.h"
#include "tagslam/yaml_utils.h"
#include <XmlRpcException.h>

namespace tagslam {
  using boost::irange;
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
    for (const auto cam_idx : irange(0ul, cameras_.size())) {
      const auto &ci = cameras_[cam_idx].intrinsics;
      tagGraph_.addCamera(cam_idx, ci.intrinsics,
                          ci.distortion_model,
                          ci.distortion_coeffs);
    }
    XmlRpc::XmlRpcValue static_objects;
    nh_.getParam("tag_poses/static_objects", static_objects);
    nh_.param<double>("default_tag_size", defaultTagSize_, 0.04);
    if (static_objects.getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
      ROS_ERROR("cannot find static_objects in yaml file!");
      return (false);
    }
    for (const auto i: irange(0, static_objects.size())) {
      if (static_objects[i].getType() !=
          XmlRpc::XmlRpcValue::TypeStruct) continue;
      //
      for (XmlRpc::XmlRpcValue::iterator it = static_objects[i].begin();
           it != static_objects[i].end(); ++it) {
        if (it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
          ROS_ERROR_STREAM("object " << it->first << " must be of type struct!");
          continue;
        }
        parseStaticObject(it->first, it->second);
      }
    }
    if (staticObjects_.empty()) {
      staticObjects_.push_back(StaticObject("world", gtsam::Pose3(),
                                            utils::make_pose_noise(1e-4, 1e-4)));
    }
    return (true);
  }

  bool TagSlam::subscribe() {
    if (cameras_.size() == 1) {
      singleCamSub_ = nh_.subscribe(cameras_[0].tagtopic, 1,
                                    &TagSlam::callback1, this);
    } else {
      for (const auto &cam : cameras_) {
        sub_.push_back(std::shared_ptr<TagSubscriber>(
                         new TagSubscriber(nh_, cam.tagtopic, 1)));
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

  int TagSlam::findTagType(double size) {
    // check if this tag is of unknown size. If yes,
    // declare it a new type.
    if (tagTypeMap_.count(size) == 0) {
      tagTypeMap_.insert(std::map<double,int>::value_type(size, tagTypeMap_.size()));
    }
    return (tagTypeMap_.find(size)->second);
  }

  void TagSlam::parseStaticObject(const std::string &name,
                                  XmlRpc::XmlRpcValue &staticObject) {
    gtsam::Pose3 pose;
    utils::PoseNoise noise;

    for (XmlRpc::XmlRpcValue::iterator it = staticObject.begin();
         it != staticObject.end(); ++it) {
      if (it->first == "pose" &&
          it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        yaml_utils::get_pose_and_noise(it->second, &pose, &noise);
        break;
      }
    }
    unsigned int objIdx  = staticObjects_.size();
    staticObjects_.push_back(StaticObject(name, pose, noise));

    for (XmlRpc::XmlRpcValue::iterator it = staticObject.begin();
         it != staticObject.end(); ++it) {
      if (it->first == "tags") {
        std::vector<Tag> tags = Tag::parseTags(it->second);
        ROS_INFO_STREAM("adding static object: " << name <<
                        " with " << tags.size() << "tags");
        for (Tag &t: tags) {
          if (idToTag_.count(t.id) != 0) {
            ROS_ERROR_STREAM("duplicate tag id in input file: " << t.id);
            ros::shutdown();
          }
          t.type = findTagType(t.size);
          t.parentIdx = objIdx;
          idToTag_.insert(IdToTagMap::value_type(t.id, t));
          ROS_INFO_STREAM("adding to static object " << name << " tag with id: " << t.id);
        }
        tagGraph_.addTags(name, pose, noise, tags);
        return;
      }
    }
    ROS_WARN_STREAM("warning: ignoring object " << name << " without tags");
  }

  static ros::Time get_latest_time(const std::vector<TagArrayConstPtr> &msgvec) {
    ros::Time t(0);
    for (const auto &m: msgvec) {
      if (m->header.stamp > t) t = m->header.stamp;
    }
    return (t);
  }

//#define DEBUG_CAM_POSE_INIT

  // computes estimated T_w_c
  bool TagSlam::estimateCameraPose(int cam_idx, const std::vector<Tag> &tags,
                                   gtsam::Pose3 *pose, double *err) {
    if (tags.empty()) {
      return (false);
    }
    const Camera &cam = cameras_[cam_idx];
#ifdef DEBUG_CAM_POSE_INIT
    std::cout << "---------- points used for cam " << cam_idx << " pose estimation: " << std::endl;
#endif    
    const auto &ci = cam.intrinsics;
    std::vector<cv::Point2d> ip;
    std::vector<cv::Point3d> wp;
    for (const auto &tag: tags) {
      for (const auto i: irange(0, 4)) {
        gtsam::Point2 uv  = tag.corners[i];
        const gtsam::Pose3  &T_w_s = staticObjects_[tag.parentIdx].pose;
        gtsam::Point3 p =  T_w_s * tag.pose * tag.getObjectCorner(i);
        wp.push_back(cv::Point3d(p.x(), p.y(), p.z()));
        ip.push_back(cv::Point2d(uv.x(), uv.y()));
#ifdef DEBUG_CAM_POSE_INIT
        std::cout << tag.id << " " << ip.back().x << " " << ip.back().y << " " << wp.back().x << " "
                  << wp.back().y << " " << wp.back().z << std::endl;
#endif
      }
    }

    gtsam::Pose3 prevPose;
    bool hasPrevPose = tagGraph_.getCameraPose(cam_idx, &prevPose);
    cv::Mat rvec, tvec;
    bool rc = utils::get_init_pose_pnp(wp, ip, ci.K, ci.distortion_model, ci.D, &rvec, &tvec);
    if (!rc) {
      if (hasPrevPose) {
        *pose = prevPose;
        return (true);
      }
      return (false);
    }
    *err = utils::reprojection_error(wp, ip, rvec, tvec, ci.K, ci.distortion_model, ci.D);
    gtsam::Vector tvec_gtsam = (gtsam::Vector(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)).finished();
    gtsam::Pose3 T_c_w_guess(gtsam::Rot3::rodriguez(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)), tvec_gtsam);
    gtsam::Pose3 T_w_c_guess = T_c_w_guess.inverse();
#ifdef DEBUG_CAM_POSE_INIT
    std::cout << "reprojection error: " << *err <<std::endl;
    std::cout << "pnp pose T_c_w guess: " << std::endl << T_c_w_guess << std::endl;
#endif
    
    if (hasPrevPose) {
      gtsam::Pose3 errPose = prevPose.between(T_w_c_guess);
#ifdef DEBUG_CAM_POSE_INIT      
      std::cout << "prev pose: " << prevPose << std::endl;
#endif
      gtsam::Vector3 er   = gtsam::Rot3::Logmap(errPose.rotation());
      gtsam::Vector3 et = errPose.translation();
      double roterr = std::sqrt(er(0)*er(0) + er(1)*er(1) + er(2)*er(2));
      double transerr = std::sqrt(et(0)*et(0) + et(1)*et(1) + et(2)*et(2));
#ifdef DEBUG_CAM_POSE_INIT
      std::cout << "rotation error: " << roterr <<  " translation error: " << transerr << std::endl;
#endif
    }
    
    *pose = T_w_c_guess;
    return (rc);
  }

  void TagSlam::updateTagPosesFromGraph(const std::vector<Tag> &tags) {
    //std::cout << "!!!!!!!! updating tag poses !!!!!!!!!!!!!!" << std::endl;
    for (const auto &tag: tags) {
      const gtsam::Pose3 T_w_o = tagGraph_.getTagWorldPose(tag.id);
      Tag &newTag = idToTag_[tag.id];
      //std::cout << "initial guess: " << std::endl << newTag.pose << std::endl;
      //std::cout << "updated to: " << std::endl << T_w_o << std::endl;
      newTag.pose = T_w_o;
    }
  }

  Tag TagSlam::makeTag(int id, double size, const gtsam::Pose3 &pose,
                       const Tag::PoseNoise &noise,
                       const geometry_msgs::Point *corners,
                       int parentIdx) {
    Tag newTag(id, findTagType(size), size, pose, noise);
    newTag.parentIdx = 0; // default to "first static object"
    newTag.setCorners(corners);
    return (newTag);
  }


  void TagSlam::findKnownTags(const TagArrayConstPtr &observedTags,
                              std::vector<Tag> *knownTags,
                              std::vector<Tag> *unknownTags) {
    for (const auto &tag: observedTags->apriltags) {
      if (idToTag_.empty()) {
        // no pre-defined tags, make first visible tag
        // the center of world!
                             
        Tag::PoseNoise tagNoise = utils::make_pose_noise(1e-4, 1e-4);
        gtsam::Pose3   tagPose;
        Tag newTag = makeTag(tag.id, defaultTagSize_, tagPose,
                             tagNoise, &tag.corners[0], 0);
        knownTags->push_back(newTag);
        const StaticObject &so = staticObjects_[0];
        tagGraph_.addTags(so.name, so.pose, so.noise, *knownTags);
        return; // ignore all other tags for now
      }
      const auto it = idToTag_.find(tag.id);
      if (it != idToTag_.end()) {
        // tag is known, just update corners
        Tag newTag(it->second);
        newTag.setCorners(&tag.corners[0]);
        knownTags->push_back(newTag);
      } else {
        // unknown tag, will determine approximate pose later
        Tag::PoseNoise tagNoise = utils::make_pose_noise(30, 10);
        Tag newTag = makeTag(tag.id, defaultTagSize_, gtsam::Pose3(),
                             tagNoise, &tag.corners[0], 0);
        unknownTags->push_back(newTag);
      }
    }
  }
        
  void TagSlam::process(const std::vector<TagArrayConstPtr> &msgvec) {
    for (const auto cam_idx: irange(0ul, msgvec.size())) {
      const auto tags = msgvec[cam_idx];
      std::vector<Tag> knownTags, unknownTags;
      findKnownTags(tags, &knownTags, &unknownTags);
      gtsam::Pose3 cameraPose;
      double err;
      // estimate camera just from the known tags
      bool havePose = estimateCameraPose(cam_idx, knownTags, &cameraPose, &err);
      if (havePose) {
        std::vector<Tag> tagsWithPoses;
        findTagInitialPoses(&tagsWithPoses, unknownTags, cam_idx, cameraPose);
        tagGraph_.addTags("unknown", gtsam::Pose3(),
                          utils::make_pose_noise(0.001, 0.001), tagsWithPoses);
        for (const auto &tag : tagsWithPoses) {
          knownTags.push_back(tag);
          idToTag_[tag.id] = tag;
        }
        // this will also run the optimizer
        tagGraph_.observedTags(cam_idx, knownTags, frameNum_, cameraPose);
        // after the optimizer has run, update
        // the tag poses one more time
        updateTagPosesFromGraph(tagsWithPoses);
        ROS_INFO_STREAM("observed " << knownTags.size()
                        << " tags on cam: " << cam_idx << " repro err: " << err
                        << " opt err: " << tagGraph_.getError()
                        << " opt iter: " << tagGraph_.getIterations());
      }
    }
    ros::Time t = get_latest_time(msgvec);
    broadcastCameraPoses(t);
    broadcastTagPoses(t);
    frameNum_++;
  }

  void TagSlam::findTagInitialPoses(std::vector<Tag> *tagsWithPoses,
                                    const std::vector<Tag> &newTags, int cam_idx,
                                    const gtsam::Pose3 &T_w_c) {
    for (auto &tag: newTags) {
      gtsam::Pose3 tagPose;
      if (estimateInitialTagPose(cam_idx, T_w_c, &tag.corners[0], &tagPose)) {
        Tag t(tag);
        t.pose = tagPose;
        tagsWithPoses->push_back(t);
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


  void TagSlam::broadcastCameraPoses(const ros::Time &t) {
    std::vector<std::pair<int, gtsam::Pose3>> camPoses;
    tagGraph_.getCameraPoses(&camPoses, frameNum_);
    std::vector<PoseInfo> camPoseInfo;
    for (const auto &cp: camPoses) {
      const std::string frame_id = "cam_" + std::to_string(cp.first);
      camPoseInfo.push_back(PoseInfo(cp.second, t, frame_id));
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

//#define DEBUG_TAG_POSE_INIT
  bool TagSlam::estimateInitialTagPose(int cam_idx, const gtsam::Pose3 &T_w_c,
                                       const gtsam::Point2 *corners,
                                       gtsam::Pose3 *tagPose) const {
    std::vector<gtsam::Point3> wp = Tag::get_object_corners(defaultTagSize_);
    std::vector<gtsam::Point2> ip;
    for (const auto i: irange(0, 4)) {
      ip.push_back(corners[i]);
    }

#ifdef DEBUG_TAG_POSE_INIT
    std::cout << "world points: " << std::endl;
    for (const auto &p: wp) {
      std::cout << p << std::endl;
    }
    std::cout << "image points: " << std::endl;
    for (const auto &p: ip) {
      std::cout << p << std::endl;
    }
#endif
    // find pose w.r.t camera
    const CameraIntrinsics &ci = cameras_[cam_idx].intrinsics;
    gtsam::Pose3 T_c_o = utils::get_init_pose(wp, ip, ci.intrinsics,
                                              ci.distortion_model,
                                              ci.distortion_coeffs);
#ifdef DEBUG_TAG_POSE_INIT    
    std::cout << "estimated T_c_o: " << std::endl << T_c_o << std::endl;
    std::cout << "camera T_w_c: "    << std::endl << T_w_c << std::endl;
#endif    
    // unknown tags belong to static object 0 by default
    const gtsam::Pose3 &T_s_w = staticObjects_[0].pose.inverse();
#ifdef DEBUG_TAG_POSE_INIT    
    std::cout << "object pose T_s_w: " << std::endl << T_s_w << std::endl;
#endif    
    const gtsam::Pose3 T_s_o = T_s_w * T_w_c * T_c_o;
    *tagPose = T_s_o;
#ifdef DEBUG_TAG_POSE_INIT
    std::cout << "tag pose T_s_o: " << std::endl << *tagPose << std::endl;
#endif
    return (true);
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

}  // namespace
