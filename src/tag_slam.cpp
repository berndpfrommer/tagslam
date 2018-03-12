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
      const auto &K  = ci.intrinsics;
      tagGraph_.addCamera(cam_idx, K[0], K[1], K[2], K[3],
                          ci.distortion_model,
                          ci.distortion_coeffs);
    }
    XmlRpc::XmlRpcValue static_objects;
    nh_.getParam("tag_poses/static_objects", static_objects);
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

  void TagSlam::process(const std::vector<TagArrayConstPtr> &msgvec) {
    for (const auto cam_idx: irange(0ul, msgvec.size())) {
      const auto tags = msgvec[cam_idx];
      std::vector<Tag> newTags, observedTags;
      if (filterTags(&newTags, &observedTags, tags)) {
#if 0
        tagGraph_.addTags("unknown", gtsam::Pose3(),
                          utils::make_pose_noise(0.001, 0.001), newTags);
#endif        
      }
      const Camera &cam = cameras_[cam_idx];
      tagGraph_.observedTags(cam_idx, observedTags, frameNum_, cam.intrinsics.K,
                             cam.intrinsics.D);
    }
    ros::Time t = get_latest_time(msgvec);
    broadcastCameraPoses(t);
    broadcastTagPoses(t);
    frameNum_++;
  }

  bool TagSlam::filterTags(std::vector<Tag> *newTags, std::vector<Tag> *observedTags,
                           TagArrayConstPtr tags) {
    for (const auto &tag: tags->apriltags) {
      if (idToTag_.count(tag.id) == 0) {
        continue; // XXX
        // must assume some default size for unknown tags
        if (tagTypeMap_.count(defaultTagSize_) == 0) {
          tagTypeMap_.insert(std::map<double,int>::value_type(
                               defaultTagSize_, tagTypeMap_.size()));
        }
        // unknown tags are given a large error prior
        Tag newTag(tag.id, tagTypeMap_[defaultTagSize_],
                   defaultTagSize_, gtsam::Pose3(),
                   utils::make_pose_noise(30.0, 1.0e3));
        newTags->push_back(newTag);
        idToTag_.insert(IdToTagMap::value_type(tag.id, newTag));
      }
      Tag newTag(idToTag_[tag.id]);
      for (int i = 0; i < 4; i++) {
        newTag.corners[i] = gtsam::Point2(tag.corners[i].x,
                                          tag.corners[i].y);
      }
      observedTags->push_back(newTag);
      break; // XXX
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
    std::vector<PoseInfo> camPoseInfo;
    for (const auto &cp: camPoses) {
      const std::string frame_id = "cam_" + std::to_string(cp.first);
      camPoseInfo.push_back(PoseInfo(cp.second, t, frame_id));
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
