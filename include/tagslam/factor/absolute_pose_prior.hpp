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

#ifndef TAGSLAM__ABSOLUTE_POSE_PRIOR_HPP_
#define TAGSLAM__ABSOLUTE_POSE_PRIOR_HPP_

#include <tagslam/factor/factor.hpp>
#include <tagslam/pose_with_noise.hpp>

namespace tagslam
{
namespace factor
{
using std::string;
class AbsolutePosePrior : public Factor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AbsolutePosePrior(
    uint64_t t = 0, const PoseWithNoise & p = PoseWithNoise(),
    const string & name = "")
  : Factor(name, t), poseWithNoise_(p)
  {
  }
  // ----- inherited methods
  string getLabel() const override;
  VertexId getId() const override { return (make_id(time_, "app_" + name_)); }
  std::shared_ptr<Vertex> clone() const override
  {
    return (std::shared_ptr<AbsolutePosePrior>(new AbsolutePosePrior(*this)));
  }
  VertexDesc addToGraph(const VertexPtr & vp, Graph * g) const override;
  void addToOptimizer(Graph * g) const override;
  bool establishesValues() const override { return (true); }
  // -------- own methods
  const PoseWithNoise & getPoseWithNoise() const { return (poseWithNoise_); }

private:
  PoseWithNoise poseWithNoise_;
};
}  // namespace factor
typedef std::shared_ptr<factor::AbsolutePosePrior> AbsolutePosePriorFactorPtr;
typedef std::shared_ptr<const factor::AbsolutePosePrior>
  AbsolutePosePriorFactorConstPtr;
}  // namespace tagslam

#endif  // TAGSLAM__ABSOLUTE_POSE_PRIOR_HPP_