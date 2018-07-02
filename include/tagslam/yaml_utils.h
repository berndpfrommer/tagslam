/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef YAML_UTILS_H
#define YAML_UTILS_H

#include "tagslam/utils.h"
#include "tagslam/pose_noise.h"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <iostream>
#include <string>

namespace tagslam {
  // computes a pose (rotation vector, translation) via homography
  // from world and image points. Distortion is not taken into account,
  // obviously this is just a starting guess.
  namespace yaml_utils {
    Eigen::Vector3d get_vec(const std::string &name, XmlRpc::XmlRpcValue v);
    bool get_pose_and_noise(XmlRpc::XmlRpcValue pose_and_noise,
                            gtsam::Pose3 *pose, PoseNoise *noise,
                            double defPosNoise = 0, double defRotNoise = 0);
    void write_pose(std::ostream &of, const std::string &prefix,
                    const gtsam::Pose3 &pose,
                    const PoseNoise &n, bool writeNoise);
    void write_pose_with_covariance(std::ostream &of,
                                    const std::string &prefix,
                                    const gtsam::Pose3 &pose,
                                    const PoseNoise &n);
    template <typename T>
    T parse(XmlRpc::XmlRpcValue xml, const std::string key, const T &def) {
      if (xml.hasMember(key)) {
        return (static_cast<T>(xml[key]));
      }
      return (def);
    }
  }
}

#endif
