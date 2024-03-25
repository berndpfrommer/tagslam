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

#ifndef TAGSLAM__COORDINATE_HPP_
#define TAGSLAM__COORDINATE_HPP_

#include <memory>
#include <tagslam/factor/factor.hpp>
#include <tagslam/geometry.hpp>
#include <tagslam/tag.hpp>
#include <tagslam/tag_factory.hpp>

namespace YAML
{
class Node;
}

namespace tagslam
{
namespace factor
{
using std::string;
class Coordinate : public Factor
{
public:
  typedef std::shared_ptr<factor::Coordinate> CoordinateFactorPtr;
  typedef std::shared_ptr<const factor::Coordinate> CoordinateFactorConstPtr;
  typedef std::vector<CoordinateFactorPtr> CoordinateFactorPtrVec;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Coordinate(
    double len, double noise, const Point3d & direction, const int corn,
    const TagConstPtr & tag, const string & name);
  // ------ inherited methods -----
  string getLabel() const override;
  VertexId getId() const override { return (name_); }
  std::shared_ptr<Vertex> clone() const override
  {
    return (std::shared_ptr<Coordinate>(new Coordinate(*this)));
  }
  VertexDesc addToGraph(const VertexPtr & vp, Graph * g) const override;
  void addToOptimizer(Graph * g) const override;
  bool establishesValues() const override { return (false); }

  // --------- own methods
  double getLength() const { return (length_); }
  double getNoise() const { return (noise_); }
  const Eigen::Vector3d & getDirection() const { return (direction_); }
  double coordinate(const Transform & T_w_b, const Transform & T_b_o) const;
  const TagConstPtr getTag() const { return (tag_); }
  const Eigen::Vector3d getCorner() const;

  // --- static methods
  inline static std::shared_ptr<const factor::Coordinate> cast_const(
    const VertexPtr & vp)
  {
    return (std::dynamic_pointer_cast<const factor::Coordinate>(vp));
  }
  inline static std::shared_ptr<factor::Coordinate> cast(const VertexPtr & vp)
  {
    return (std::dynamic_pointer_cast<factor::Coordinate>(vp));
  }
  static CoordinateFactorPtrVec parse(const YAML::Node & meas, TagFactory * tf);
  static double getOptimized(const VertexDesc & v, const Graph & g);

private:
  static CoordinateFactorPtr parse(
    const string & name, const YAML::Node & meas, TagFactory * factory);
  double length_;
  double noise_;
  Eigen::Vector3d direction_;
  int corner_;
  TagConstPtr tag_;
};
}  // namespace factor
typedef factor::Coordinate::CoordinateFactorPtr CoordinateFactorPtr;
typedef factor::Coordinate::CoordinateFactorConstPtr CoordinateFactorConstPtr;
typedef factor::Coordinate::CoordinateFactorPtrVec CoordinateFactorPtrVec;
}  // namespace tagslam
#endif  //  TAGSLAM__COORDINATE_HPP_