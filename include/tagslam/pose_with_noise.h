/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/geometry.h"
#include "tagslam/pose_noise.h"
#include <ros/ros.h>

namespace tagslam {
  class PoseWithNoise {
  public:
    using string = std::string;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PoseWithNoise(const Transform &tf = Transform::Identity(),
                  const PoseNoise &n = PoseNoise(),
                  bool isVal = false) :
      pose_(tf), noise_(n), valid_(isVal) {
    };
    const Transform  &getPose()  const { return (pose_); }
    const PoseNoise  &getNoise() const { return (noise_); }
    bool              isValid()  const { return (valid_); }
    
    void  setNoise(const PoseNoise &pn) { noise_ = pn; }

  private:
    Transform  pose_;
    PoseNoise  noise_;
    bool       valid_{false};
  };
  std::ostream &operator<<(std::ostream &os, const PoseWithNoise &pe);
}
