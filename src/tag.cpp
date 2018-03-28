/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/yaml_utils.h"
#include "tagslam/tag.h"
#include <ros/ros.h>
#include <map>

using std::cout;
using std::endl;

namespace tagslam {
  typedef std::map<double, int> SizeMap;
  static SizeMap tag_size_map;
  
  static int find_tag_type(double size) {
    // check if this tag is of unknown size. If yes,
    // declare it a new type.
    if (tag_size_map.count(size) == 0) {
      tag_size_map.insert(SizeMap::value_type(size, tag_size_map.size()));
    }
    return (tag_size_map.find(size)->second);
  }

  Tag::Tag(int ida, int tp, double s, const PoseEstimate &pe,
           bool hasKPose) :
    id(ida), type(tp), size(s), poseEstimate(pe), hasKnownPose(hasKPose) {
    objectCorners = make_object_corners(size);
    imageCorners.resize(4);
  }

  gtsam::Point3 Tag::getObjectCorner(int i) const {
    return objectCorners[i];
  }

  bool Tag::hasValidImageCorners() const {
    for (int i = 0; i < 4; i++) {
      if (imageCorners[i].x() != 0 || imageCorners[i].y() != 0) {
        return (true);
      }
    }
    return (false);
  }

  void Tag::setImageCorners(const geometry_msgs::Point *corn) {
    for (int i = 0; i < 4; i++) {
      imageCorners[i] = gtsam::Point2(corn[i].x, corn[i].y);
    }
  }
  
  gtsam::Point3 Tag::getWorldCorner(int i) const {
    return (poseEstimate.transform_from(getObjectCorner(i)));
  }

  TagVec Tag::parseTags(XmlRpc::XmlRpcValue xmltags, double size) {
    std::vector<TagPtr> tags;
    for (uint32_t i = 0; i < xmltags.size(); i++) {
      if (xmltags[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;
      int id(0);
      double sz(size), uc(0);
      for (XmlRpc::XmlRpcValue::iterator it = xmltags[i].begin();
           it != xmltags[i].end(); ++it) {
        std::string field = it->first;
        if (field == "id") {           id = static_cast<int>(it->second);
        } else  if (field == "size") { sz = static_cast<double>(it->second);
        }
      }
      gtsam::Pose3 pose;
      PoseNoise noise;
      if (yaml_utils::get_pose_and_noise(xmltags[i], &pose, &noise)) {
        PoseEstimate pe(pose, 0.0, 0, noise);
        tags.push_back(makeTag(id, sz, pe, true));
      } else {
        tags.push_back(makeTag(id, sz, PoseEstimate(), false));
      }
    }
    /*
      std::cout << "-------tags: " << std::endl;
      for (const auto &tag:tags) {
      std::cout << *tag << std::endl;
      }
    */
    return (tags);
  }

  TagPtr Tag::makeTag(int tagId, double size, const PoseEstimate &pe,
                      bool hasKnownPose) {
    TagPtr tagPtr(new Tag(tagId, find_tag_type(size),
                       size, pe, hasKnownPose));
    return (tagPtr);
  }
  
  std::vector<gtsam::Point3> Tag::make_object_corners(double size) {
    const double s = size;
    const std::vector<gtsam::Point3> c =
      {gtsam::Point3(-s/2,-s/2, 0),
       gtsam::Point3( s/2,-s/2, 0),
       gtsam::Point3( s/2, s/2, 0),
       gtsam::Point3(-s/2, s/2, 0)};
    return (c);
  }
  std::ostream &operator<<(std::ostream &os, const Tag &tag) {
    os << tag.id << " sz: " << tag.size << " ty: " << tag.type << " " << tag.poseEstimate;
    return (os);
  }
}  // namespace
