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
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      Pose(const ros::Time    &t    = ros::Time(0),
           const Transform    &p    = Transform::Identity(),
           const std::string  &name = "",
           bool               valid = false) :
        Value(name, valid), time_(t), pose_(p) {}
      std::string getLabel() const override;
      const Transform &getPose() const { return (pose_); }
      // ---- methods for optimizer adding
      void addToOptimizer(Optimizer *opt,
                          const BoostGraph::vertex_descriptor &v,
                          const BoostGraph *g) override;
    private:
      ros::Time time_;
      Transform pose_;
    };
  }
  typedef std::shared_ptr<value::Pose> PoseValuePtr;
  typedef std::shared_ptr<const value::Pose> PoseValueConstPtr;
}
