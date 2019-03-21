/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/factor/factor.h"
#include "tagslam/pose_with_noise.h"
#include <ros/ros.h>

namespace tagslam {
  namespace factor {
    class AbsolutePosePrior: public Factor {
    public:
      AbsolutePosePrior(const ros::Time     &t  = ros::Time(0),
                        const PoseWithNoise &p  = PoseWithNoise(),
                        const std::string   &name = "") :
        Factor(name, t), poseWithNoise_(p) {
        setIsValid(p.isValid());
      }
      std::string getLabel() const override;
    private:
      PoseWithNoise poseWithNoise_;
    };
  }
}
