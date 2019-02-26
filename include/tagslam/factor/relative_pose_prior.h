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
        Factor(name), time_(t), poseWithNoise_(p) {}
      std::string getLabel() const override;
      // ---- methods for optimizer adding
      void addToOptimizer(Optimizer *opt,
                          const BoostGraph::vertex_descriptor &v,
                          const BoostGraph *g) override;
    private:
      ros::Time     time_;
      PoseWithNoise poseWithNoise_;
    };
  }
}
