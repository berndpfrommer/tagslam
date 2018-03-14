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
          // check if this tag is of unknown size. If yes,
          // declare it a new type.
          if (tagTypeMap_.count(t.size) == 0) {
            tagTypeMap_.insert(std::map<double,int>::value_type(t.size, tagTypeMap_.size()));
          }
          t.type = tagTypeMap_[t.size];
          t.parentIdx = objIdx;
          idToTag_.insert(IdToTagMap::value_type(t.id, t));
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

  bool TagSlam::estimateCameraPose(const Camera &cam, const std::vector<Tag> &tags,
                                   gtsam::Pose3 *pose) {
    if (tags.empty()) {
      return (false);
    }
    const auto &ci = cam.intrinsics;
    std::vector<cv::Point2f> ip;
    std::vector<cv::Point3f> wp;
    for (const auto &tag: tags) {
      for (const auto i: irange(0, 4)) {
        gtsam::Point2 uv  = tag.corners[i];
        const gtsam::Pose3  &T_w_s = staticObjects_[tag.parentIdx].pose;
        gtsam::Point3 p =  T_w_s * tag.pose * tag.getObjectCorner(i);
        wp.push_back(cv::Point3f(p.x(), p.y(), p.z()));
        ip.push_back(cv::Point2f(uv.x(), uv.y()));
      }
    }
#if 0
    std::cout << "---------- points used for cam pose estimation: " << std::endl;
    for (const auto i: irange(0ul, ip.size())) {
      std::cout << i << " " << ip[i] << " " << wp[i] << std::endl;
    }
#endif    

    bool rc(true);
    gtsam::Pose3 T_w_c_guess = utils::get_init_pose_pnp(wp, ip, ci.K, ci.D, &rc);
    *pose = T_w_c_guess;
    return (rc);
  }

  void TagSlam::process(const std::vector<TagArrayConstPtr> &msgvec) {
    for (const auto cam_idx: irange(0ul, msgvec.size())) {
      const auto tags = msgvec[cam_idx];
      std::vector<Tag> newTags, observedTags;
      if (filterTags(cam_idx, &newTags, &observedTags, tags)) {
        tagGraph_.addTags("unknown", gtsam::Pose3(),
                          utils::make_pose_noise(0.001, 0.001), newTags);
      }
      std::cout << "out of " << tags->apriltags.size() << " useful tags: " << observedTags.size() << std::endl;

      gtsam::Pose3 cameraPose;
      if (estimateCameraPose(cameras_[cam_idx], observedTags, &cameraPose)) {
        ROS_INFO_STREAM("have pose and useful tags!");
        tagGraph_.observedTags(cam_idx, observedTags, frameNum_, cameraPose);
      } else {
        std::cout << "no camera pose found!" << std::endl;
      }
    }
    ros::Time t = get_latest_time(msgvec);
    broadcastCameraPoses(t);
    broadcastTagPoses(t);
    frameNum_++;
  }

  bool TagSlam::filterTags(int cam_idx, std::vector<Tag> *newTags,
                           std::vector<Tag> *observedTags,
                           TagArrayConstPtr tags) {
    gtsam::Pose3 T_w_c;
    bool hasValidCameraPose = tagGraph_.getCameraPose(cam_idx, &T_w_c);

    for (const auto &tag: tags->apriltags) {
      bool isNewTag(false);
      if (idToTag_.count(tag.id) == 0) { // new, unknown tag
        ROS_INFO_STREAM("found unknown tag with id: " << tag.id << " for cam: " << cam_idx);
        gtsam::Pose3 tagPose;
        Tag::PoseNoise tagNoise = utils::make_pose_noise(30.0, 1.0e3);
        if (!hasValidCameraPose || !estimateInitialTagPose(cam_idx, T_w_c,
                                                           &tag.corners[0], &tagPose)) {
          ROS_INFO_STREAM("cannot estimate tag pose for tag " << tag.id);
          // hmm, cannot estimate tag pose. If no other tags are
          // already there with known position (i.e. pre-defined in file,
          // or found earlier), this tag becomes the center of the universe!
          if (idToTag_.empty()) {
            ROS_INFO_STREAM("first tag observed, making tag " << tag.id << " center of universe!");
            tagPose = gtsam::Pose3(); // not really necessary ....
            tagNoise = utils::make_pose_noise(1e-4, 1e-4);
          } else {
            ROS_INFO_STREAM("ignoring tag " << tag.id);
            // TODO: the continue is tricky code, don't like it
            continue; // ignore new tag if we cannot locate it!
          }
        }
        // ok, we have a new tag, and we have a clue about its pose.
        // enter it into the tag database
        Tag newTag(tag.id, tagTypeMap_[defaultTagSize_],
                   defaultTagSize_, tagPose, tagNoise);
        idToTag_.insert(IdToTagMap::value_type(tag.id, newTag));
        isNewTag = true;
      } else {
        ROS_INFO_STREAM("found known tag with id: " << tag.id << " for cam: " << cam_idx);
      }
      // at this point we have a valid tag, and we
      // know its pose. Let's put the corners on it as well
      Tag &obsTag = idToTag_.find(tag.id)->second;
      for (int i = 0; i < 4; i++) {
        obsTag.corners[i] = gtsam::Point2(tag.corners[i].x,
                                          tag.corners[i].y);
      }
      observedTags->push_back(obsTag);
      if (isNewTag) {
        newTags->push_back(obsTag);
      }
    }
    return (!newTags->empty());
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
    std::cout << "got camera positions: " << camPoses.size() << std::endl;
    std::vector<PoseInfo> camPoseInfo;
    for (const auto &cp: camPoses) {
      const std::string frame_id = "cam_" + std::to_string(cp.first);
      camPoseInfo.push_back(PoseInfo(cp.second, t, frame_id));
      std::cout << "publishing camera position: " << frame_id << std::endl;
    }
    broadcastTransforms(camPoseInfo);
  }

  void TagSlam::broadcastTagPoses(const ros::Time &t) {
    std::vector<std::pair<int, gtsam::Pose3>> tagPoses;
    tagGraph_.getTagPoses(&tagPoses);
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
    
  bool TagSlam::estimateInitialTagPose(int cam_idx, const gtsam::Pose3 &T_w_c,
                                       const geometry_msgs::Point *corners,
                                       gtsam::Pose3 *pose) const {
    std::vector<gtsam::Point3> wp = Tag::get_object_corners(defaultTagSize_);
    std::vector<gtsam::Point2> ip;
    for (const auto i: irange(0, 4)) {
      ip.emplace_back(corners[i].x, corners[i].y);
    }

#if 0
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
    //std::cout << "estimated T_c_o: " << std::endl << T_c_o << std::endl;
    //std::cout << "camera T_w_c: "    << std::endl << T_w_c << std::endl;
    // unknown tags belong to static object 0 by default
    const gtsam::Pose3 &T_s_w = staticObjects_[0].pose.inverse();
    //std::cout << "object pose T_s_w: " << std::endl << T_s_w << std::endl;
    const gtsam::Pose3 T_s_o = T_s_w * T_w_c * T_c_o;
    *pose = T_s_o;
    //std::cout << "tag pose T_s_o: " << std::endl << *pose << std::endl;
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
