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
           const std::string  &name = "",
           bool                 icp = false) :
        Value(name, t), isCameraPose_(icp) {}
      std::string getLabel() const override;
      std::shared_ptr<Vertex> clone() const override {
        return (std::shared_ptr<Pose>(new Pose(*this))); }
      VertexId getId() const override { return (id(time_, name_));}
      VertexDesc addToGraph(const VertexPtr &vpk, Graph *g) const override;
      void addToOptimizer(const Transform &tf, Graph *g) const;

      bool isCameraPose() const { return (isCameraPose_); }
      void setIsCameraPose(bool c) {  isCameraPose_ = c; }
      static VertexId id(const ros::Time &t, const std::string &n) {
        return (make_id(t, n));
      }
    private:
      bool      isCameraPose_{false};
    };
  }
  typedef std::shared_ptr<value::Pose> PoseValuePtr;
  typedef std::shared_ptr<const value::Pose> PoseValueConstPtr;
}
