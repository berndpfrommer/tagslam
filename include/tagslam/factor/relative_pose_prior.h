/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/factor/factor.h"
#include "tagslam/pose_with_noise.h"
#include <ros/ros.h>

namespace tagslam {
  namespace factor {
    class RelativePosePrior: public Factor {
    public:
      RelativePosePrior(const ros::Time     &t  = ros::Time(0),
                        const PoseWithNoise &p  = PoseWithNoise(),
                        const std::string   &name = "") :
        Factor(name, t), poseWithNoise_(p) {}
      std::string getLabel() const override;
      const PoseWithNoise &getPoseWithNoise() const { return (poseWithNoise_); }
    private:
      PoseWithNoise poseWithNoise_;
    };
  }
  typedef std::shared_ptr<factor::RelativePosePrior> RelativePosePriorFactorPtr;
  typedef std::shared_ptr<const factor::RelativePosePrior> RelativePosePriorFactorConstPtr;
}
