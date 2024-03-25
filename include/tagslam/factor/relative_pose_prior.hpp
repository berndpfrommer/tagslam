// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TAGSLAM__RELATIVE_POSE_PRIOR_HPP_
#define TAGSLAM__RELATIVE_POSE_PRIOR_HPP_

#include <tagslam/factor/factor.hpp>
#include <tagslam/pose_with_noise.hpp>

namespace tagslam
{
namespace factor
{
using std::string;
class RelativePosePrior : public Factor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RelativePosePrior(
    uint64_t t = 0, uint64_t tm1 = 0, const PoseWithNoise & p = PoseWithNoise(),
    const string & name = "")
  : Factor(name, t), prevTime_(tm1), poseWithNoise_(p)
  {
  }
  // ---------- inherited
  string getLabel() const override;
  VertexId getId() const override { return (make_id(time_, "rpp_" + name_)); }
  std::shared_ptr<Vertex> clone() const override
  {
    return (std::shared_ptr<RelativePosePrior>(new RelativePosePrior(*this)));
  }
  VertexDesc addToGraph(const VertexPtr & vp, Graph * g) const override;
  void addToOptimizer(Graph * g) const override;
  bool establishesValues() const override { return (true); }
  // ---------- own methods
  uint64_t getPreviousTime() const { return (prevTime_); }
  const PoseWithNoise & getPoseWithNoise() const { return (poseWithNoise_); }

private:
  uint64_t prevTime_;
  PoseWithNoise poseWithNoise_;
};
}  // namespace factor
typedef std::shared_ptr<factor::RelativePosePrior> RelativePosePriorFactorPtr;
typedef std::shared_ptr<const factor::RelativePosePrior>
  RelativePosePriorFactorConstPtr;
}  // namespace tagslam

#endif  // TAGSLAM__RELATIVE_POSE_PRIOR_HPP_