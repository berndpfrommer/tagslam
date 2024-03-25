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

#ifndef TAGSLAM__TAGSLAM__POSE_HPP_
#define TAGSLAM__TAGSLAM__POSE_HPP_

#include <tagslam/pose_with_noise.hpp>
#include <tagslam/value/value.hpp>

namespace tagslam
{
namespace value
{
class Pose : public Value
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Pose(const uint64_t t = 0, const std::string & name = "", bool icp = false)
  : Value(name, t), isCameraPose_(icp)
  {
  }
  std::string getLabel() const override;
  std::shared_ptr<Vertex> clone() const override
  {
    return (std::shared_ptr<Pose>(new Pose(*this)));
  }
  VertexId getId() const override { return (id(time_, name_)); }
  VertexDesc addToGraph(const VertexPtr & vpk, Graph * g) const override;
  void addToOptimizer(const Transform & tf, Graph * g) const;

  bool isCameraPose() const { return (isCameraPose_); }
  void setIsCameraPose(bool c) { isCameraPose_ = c; }
  static VertexId id(uint64_t t, const std::string & n)
  {
    return (make_id(t, n));
  }

private:
  bool isCameraPose_{false};
};
}  // namespace value
typedef std::shared_ptr<value::Pose> PoseValuePtr;
typedef std::shared_ptr<const value::Pose> PoseValueConstPtr;
}  // namespace tagslam

#endif  // TAGSLAM__TAGSLAM__POSE_HPP_