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
        Value(name, t, valid), pose_(p) {}
      std::string getLabel() const override;
      std::shared_ptr<Vertex> clone() const override {
        return (std::shared_ptr<Pose>(new Pose(*this))); }
      VertexId getId() const override { return (id(time_, name_));}
      VertexDesc attachTo(Graph *g) const override;
      void addToOptimizer(Graph *g) override;

      const Transform &getPose() const { return (pose_); }
      void setPose(const Transform &pose) {
        pose_  = pose;
        setIsValid(true);
      }
      static VertexId id(const ros::Time &t, const std::string &n) {
        return (make_id(t, n));
      }
    private:
      Transform pose_;
    };
  }
  typedef std::shared_ptr<value::Pose> PoseValuePtr;
  typedef std::shared_ptr<const value::Pose> PoseValueConstPtr;
}
