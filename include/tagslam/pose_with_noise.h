/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/geometry.h"
#include "tagslam/pose_noise2.h"
#include <ros/ros.h>

namespace tagslam {
  class PoseWithNoise {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PoseWithNoise(const Transform &tf = Transform::Identity(),
                  const PoseNoise2 &n = PoseNoise2(),
                  bool isVal = false) :
      pose(tf), noise(n), valid(isVal) {
    };
    const Transform  &getPose()  const { return (pose); }
    const PoseNoise2 &getNoise() const { return (noise); }
    bool              isValid()  const { return (valid); }
    static PoseWithNoise parse(const std::string &name,
                               const ros::NodeHandle &nh);
  private:
    Transform  pose;
    PoseNoise2 noise;
    bool       valid{false};
  };
  std::ostream &operator<<(std::ostream &os, const PoseWithNoise &pe);
}
