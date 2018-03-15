/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/yaml_utils.h"
#include "tagslam/tag.h"
#include <ros/ros.h>

using std::cout;
using std::endl;

namespace tagslam {
  gtsam::Point3 Tag::getObjectCorner(int i) const {
    double s = size;
    const gtsam::Point3 c[4] = {gtsam::Point3(-s/2,-s/2, 0),
                                gtsam::Point3( s/2,-s/2, 0),
                                gtsam::Point3( s/2, s/2, 0),
                                gtsam::Point3(-s/2, s/2, 0)};
    return (c[i]);
  }

  std::vector<gtsam::Point3> Tag::get_object_corners(double size) {
    const double s = size;
    const std::vector<gtsam::Point3> c =
      {gtsam::Point3(-s/2,-s/2, 0),
       gtsam::Point3( s/2,-s/2, 0),
       gtsam::Point3( s/2, s/2, 0),
       gtsam::Point3(-s/2, s/2, 0)};
    return (c);
  }

  std::vector<gtsam::Point3> Tag::getObjectCorners() const {
    return (get_object_corners(size));
  }
  
  std::vector<gtsam::Point2> Tag::getImageCorners() const {
    std::vector<gtsam::Point2> v;
    for (int i = 0; i < 4; i++) {
      v.push_back(corners[i]);
    }
    return (v);
  }
  bool Tag::hasValidImageCorners() const {
    for (int i = 0; i < 4; i++) {
      if (corners[i].x() != 0 || corners[i].y() != 0) {
        return (true);
      }
    }
    return (false);
  }

  void Tag::setCorners(const geometry_msgs::Point *corn) {
    for (int i = 0; i < 4; i++) {
      corners[i] = gtsam::Point2(corn[i].x, corn[i].y);
    }
  }
  
  gtsam::Point3 Tag::getWorldCorner(int i) const {
    return (pose.transform_from(getObjectCorner(i)));
  }

  std::vector<Tag> Tag::parseTags(XmlRpc::XmlRpcValue poses) {
    std::vector<Tag> tags;
    for (uint32_t i = 0; i < poses.size(); i++) {
      if (poses[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;
      int id(0);
      double sz(0.1), uc(0);
      for (XmlRpc::XmlRpcValue::iterator it = poses[i].begin();
           it != poses[i].end(); ++it) {
        std::string field = it->first;
        if (field == "id") {           id = static_cast<int>(it->second);
        } else  if (field == "size") { sz = static_cast<double>(it->second);
        } else  if (field == "uncertainty") {
          uc = static_cast<double>(it->second);
        }
      }
      gtsam::Pose3 pose;
      utils::PoseNoise noise;
      yaml_utils::get_pose_and_noise(poses[i], &pose, &noise);
      Tag t(id, 0 /*num*/, sz,   pose, noise);

      tags.push_back(t);
    }
    return (tags);
  }
}  // namespace
