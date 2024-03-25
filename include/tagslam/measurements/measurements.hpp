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

#ifndef TAGSLAM__MEASUREMENTS_HPP_
#define TAGSLAM__MEASUREMENTS_HPP_

#include <memory>
#include <tagslam/graph.hpp>
#include <tagslam/tag_factory.hpp>
#include <vector>

namespace YAML
{
class Node;  // forward decl
}

namespace tagslam
{
namespace measurements
{
class Measurements
{
public:
  virtual ~Measurements() {}
  typedef std::shared_ptr<Measurements> MeasurementsPtr;
  typedef std::shared_ptr<const Measurements> MeasurementsConstPtr;

  virtual void addToGraph(const GraphPtr & graph);
  virtual void tryAddToOptimizer(const GraphPtr & graph);
  virtual void printUnused(const GraphConstPtr & graph);

  virtual void writeDiagnostics(const GraphPtr & graph) = 0;

protected:
  template <typename T>
  static std::shared_ptr<T> cast(const FactorPtr & factor)
  {
    std::shared_ptr<T> fp = std::dynamic_pointer_cast<T>(factor);
    if (!fp) {
      throw(std::runtime_error("wrong factor type"));
    }
    return (fp);
  }
  std::vector<VertexDesc> vertexes_;
  std::vector<FactorPtr> factors_;
};
typedef Measurements::MeasurementsPtr MeasurementsPtr;
typedef Measurements::MeasurementsConstPtr MeasurementsConstPtr;
std::vector<MeasurementsPtr> read_all(
  const YAML::Node & config, TagFactory * tagFactory);
}  // namespace measurements
typedef measurements::MeasurementsPtr MeasurementsPtr;
typedef measurements::MeasurementsConstPtr MeasurementsConstPtr;
}  // namespace tagslam
#endif  // TAGSLAM__MEASUREMENTS_HPP_