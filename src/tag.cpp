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
