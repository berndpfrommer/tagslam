/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/factor/factor.h"
#include "tagslam/pose_with_noise.h"
#include <ros/ros.h>

namespace tagslam {
  namespace factor {
    using std::string;
    class RelativePosePrior: public Factor {
    public:
      RelativePosePrior(const ros::Time     &t    = ros::Time(0),
                        const ros::Time     &tm1  = ros::Time(0),
                        const PoseWithNoise &p  = PoseWithNoise(),
                        const string   &name = "") :
        Factor(name, t), prevTime_(tm1), poseWithNoise_(p) {}
      // ---------- inherited
      string getLabel() const override;
      VertexId getId() const override { return (make_id(time_, "rpp_"+name_));}
      std::shared_ptr<Vertex> clone() const override {
        return (std::shared_ptr<RelativePosePrior>(
                  new RelativePosePrior(*this))); }
      VertexDesc addToGraph(const VertexPtr &vp, Graph *g) const override;
      void addToOptimizer(Graph *g) const override;
      bool establishesValues() const override { return (true); }
      // ---------- own methods
      const ros::Time &getPreviousTime() const { return (prevTime_); }
      const PoseWithNoise &getPoseWithNoise() const {return (poseWithNoise_);}
    private:
      ros::Time     prevTime_;
      PoseWithNoise poseWithNoise_;
    };
  }
  typedef
  std::shared_ptr<factor::RelativePosePrior> RelativePosePriorFactorPtr;
  typedef
  std::shared_ptr<const factor::RelativePosePrior> RelativePosePriorFactorConstPtr;
}
