/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/yaml_utils.h"

namespace tagslam {
  namespace yaml_utils {
    Eigen::Vector3d get_vec(XmlRpc::XmlRpcValue v) {
      double x(0), y(0), z(0);
      for (XmlRpc::XmlRpcValue::iterator it = v.begin();
           it != v.end(); ++it) {
        std::string field = it->first;
        if (field == "x") {        x = static_cast<double>(it->second);
        } else if (field == "y") { y = static_cast<double>(it->second);
        } else if (field == "z") { z = static_cast<double>(it->second);
        }
      }
      return (Eigen::Vector3d(x, y, z));
    }

    void get_pose_and_noise(XmlRpc::XmlRpcValue pose_and_noise,
                            gtsam::Pose3 *pose, utils::PoseNoise *noise) {
      Eigen::Vector3d anglevec, center, rotnoise, posnoise;
      for (XmlRpc::XmlRpcValue::iterator it = pose_and_noise.begin();
           it != pose_and_noise.end(); ++it) {
        if (it->first == "rotvec") { anglevec = get_vec(it->second);
        } else if (it->first == "center") { center   = get_vec(it->second);
        } else if (it->first == "rotation_noise") { rotnoise = get_vec(it->second);
        } else if (it->first == "position_noise") { posnoise = get_vec(it->second);
        }
      }
      gtsam::Rot3   R(utils::rotmat(anglevec));
      gtsam::Point3 T(center);
      *pose = gtsam::Pose3(R, T);
      *noise = utils::make_pose_noise(rotnoise, posnoise);
    }

  }
}  // namespace
