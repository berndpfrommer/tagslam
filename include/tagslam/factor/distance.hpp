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

#ifndef TAGSLAM__DISTANCE_HPP_
#define TAGSLAM__DISTANCE_HPP_

#include <string>
#include <tagslam/factor/factor.hpp>
#include <tagslam/geometry.hpp>
#include <tagslam/tag.hpp>
#include <tagslam/tag_factory.hpp>

namespace YAML
{
class Node;  // forward decl
}

namespace tagslam
{
namespace factor
{
using std::string;
class Distance : public Factor
{
public:
  typedef std::shared_ptr<factor::Distance> DistanceFactorPtr;
  typedef std::shared_ptr<const factor::Distance> DistanceFactorConstPtr;
  typedef std::vector<DistanceFactorPtr> DistanceFactorPtrVec;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Distance(
    double dist, double noise, const int corn1, const TagConstPtr & tag1,
    const int corn2, const TagConstPtr & tag2, const string & name);
  // ------ inherited methods -----
  string getLabel() const override;
  VertexId getId() const override { return (name_); }
  std::shared_ptr<Vertex> clone() const override
  {
    return (std::shared_ptr<Distance>(new Distance(*this)));
  }
  VertexDesc addToGraph(const VertexPtr & vp, Graph * g) const override;

  void addToOptimizer(Graph * g) const override;
  bool establishesValues() const override { return (false); }

  // --------- own methods
  double getDistance() const { return (distance_); }
  double getNoise() const { return (noise_); }
  double distance(
    const Transform & T_w_b1, const Transform & T_b1_o,
    const Transform & T_w_b2, const Transform & T_b2_o) const;
  const TagConstPtr getTag(int idx) const { return (tag_[idx]); }
  const Eigen::Vector3d getCorner(int idx) const;
  // --- static methods
  inline static std::shared_ptr<const factor::Distance> cast_const(
    const VertexPtr & vp)
  {
    return (std::dynamic_pointer_cast<const factor::Distance>(vp));
  }
  inline static std::shared_ptr<factor::Distance> cast(const VertexPtr & vp)
  {
    return (std::dynamic_pointer_cast<factor::Distance>(vp));
  }
  static DistanceFactorPtrVec parse(const YAML::Node & meas, TagFactory * tf);
  static double getOptimized(const VertexDesc & v, const Graph & g);

private:
  static DistanceFactorPtr parse(
    const string & name, const YAML::Node & meas, TagFactory * factory);
  double distance_;
  double noise_;
  TagConstPtr tag_[2];
  int corner_[2];
};
}  // namespace factor
typedef factor::Distance::DistanceFactorPtr DistanceFactorPtr;
typedef factor::Distance::DistanceFactorConstPtr DistanceFactorConstPtr;
typedef factor::Distance::DistanceFactorPtrVec DistanceFactorPtrVec;
}  // namespace tagslam

#endif  // TAGSLAM__DISTANCE_HPP_