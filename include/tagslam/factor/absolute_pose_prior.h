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
      // ----- inherited methods
      std::string getLabel() const override;
      VertexId getId() const override { return (make_id(time_, "app_" + name_));}
      std::shared_ptr<Vertex> clone() const override {
        return (std::shared_ptr<AbsolutePosePrior>(new AbsolutePosePrior(*this))); }
      VertexDesc attachTo(Graph *g) const override;
      void addToOptimizer(Graph *g) override;
      bool isOptimized() const override { return (is_valid(key_)); }
      std::vector<FactorKey> getKeys()  const override {
        return (std::vector<FactorKey>(1, key_));
      }
      void clearKeys() override { key_ = Invalid; }
      // -------- own methods
      void setKey(FactorKey k) { key_ = k; }
      FactorKey getKey()  const { return (key_); }
      const PoseWithNoise &getPoseWithNoise() const { return (poseWithNoise_); }
    private:
      FactorKey key_{Invalid};
      PoseWithNoise poseWithNoise_;
    };
  }
  typedef std::shared_ptr<factor::AbsolutePosePrior> AbsolutePosePriorFactorPtr;
  typedef std::shared_ptr<const factor::AbsolutePosePrior> AbsolutePosePriorFactorConstPtr;
}
