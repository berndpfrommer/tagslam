/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/value/value.h"
#include "tagslam/pose_with_noise.h"
#include <ros/ros.h>

namespace tagslam {
  namespace value {
    class Pose: public Value {
    public:
      Pose(const ros::Time     &t   = ros::Time(0),
           const PoseWithNoise &pn  = PoseWithNoise(),
           const std::string   &name = "") :
        Value(name), time(t), poseWithNoise(pn) {}
      std::string getLabel() const override;
      // ---- methods for optimizer adding
      void addToOptimizer(GTSAMOptimizer *opt,
                          const BoostGraph::vertex_descriptor &v,
                          const BoostGraph *g) override;
    private:
      ros::Time       time;
      PoseWithNoise   poseWithNoise;
    };
  }
}
