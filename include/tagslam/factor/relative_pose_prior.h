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
      RelativePosePrior(const ros::Time     &t    = ros::Time(0),
                        const ros::Time     &tm1  = ros::Time(0),
                        const PoseWithNoise &p  = PoseWithNoise(),
                        const std::string   &name = "") :
        Factor(name, t), prevTime_(tm1), poseWithNoise_(p) {}
      // ---------- inherited
      std::string getLabel() const override;
      VertexId getId() const override { return (make_id(time_, "rpp_" + name_));}
      std::shared_ptr<Vertex> clone() const override {
        return (std::shared_ptr<RelativePosePrior>(new RelativePosePrior(*this))); }
      VertexDesc attachTo(Graph *g) const override;
      void addToOptimizer(Graph *g) override;
      bool isOptimized() const override { return (is_valid(key_)); }
      std::vector<FactorKey> getKeys()  const override {
        return (std::vector<FactorKey>(1, key_));
      }
      void clearKeys() override { key_ = Invalid; }
      // ---------- own methods
      void setKey(FactorKey k) { key_ = k; }
      FactorKey getKey()  const { return (key_); }
      const ros::Time &getPreviousTime() const { return (prevTime_); }
      const PoseWithNoise &getPoseWithNoise() const { return (poseWithNoise_); }
    private:
      ros::Time     prevTime_;
      FactorKey     key_{Invalid};
      PoseWithNoise poseWithNoise_;
    };
  }
  typedef std::shared_ptr<factor::RelativePosePrior> RelativePosePriorFactorPtr;
  typedef std::shared_ptr<const factor::RelativePosePrior> RelativePosePriorFactorConstPtr;
}
