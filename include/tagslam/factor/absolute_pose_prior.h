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
    class AbsolutePosePrior: public Factor {
    public:
      AbsolutePosePrior(const ros::Time     &t  = ros::Time(0),
                        const PoseWithNoise &p  = PoseWithNoise(),
                        const string   &name = "") :
        Factor(name, t), poseWithNoise_(p) {
      }
      // ----- inherited methods
      string getLabel() const override;
      VertexId getId() const override {
        return (make_id(time_, "app_" + name_));}
      std::shared_ptr<Vertex> clone() const override {
        return (std::shared_ptr<AbsolutePosePrior>(
                  new AbsolutePosePrior(*this))); }
      VertexDesc addToGraph(const VertexPtr &vp, Graph *g) const override;
      void addToOptimizer(Graph *g) const override;
      bool establishesValues() const override { return (true); }
      // -------- own methods
      const PoseWithNoise &getPoseWithNoise() const { return (poseWithNoise_);}
    private:
      PoseWithNoise poseWithNoise_;
    };
  }
  typedef std::shared_ptr<factor::AbsolutePosePrior>
  AbsolutePosePriorFactorPtr;
  typedef std::shared_ptr<const factor::AbsolutePosePrior>
  AbsolutePosePriorFactorConstPtr;
}
