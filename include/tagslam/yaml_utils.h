/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#pragma once

#include "tagslam/pose_noise2.h"
#include "tagslam/geometry.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <iostream>
#include <string>

namespace tagslam {
  // computes a pose (rotation vector, translation) via homography
  // from world and image points. Distortion is not taken into account,
  // obviously this is just a starting guess.
  namespace yaml_utils {
    void write_pose(std::ostream &of, const std::string &prefix,
                    const Transform &pose,
                    const PoseNoise2 &n, bool writeNoise);
    void write_pose_with_covariance(std::ostream &of,
                                    const std::string &prefix,
                                    const Transform &pose,
                                    const PoseNoise2 &n);
    template <typename T>
    T parse(XmlRpc::XmlRpcValue xml, const std::string key, const T &def) {
      if (xml.hasMember(key)) {
        return (static_cast<T>(xml[key]));
      }
      return (def);
    }
  }
}
