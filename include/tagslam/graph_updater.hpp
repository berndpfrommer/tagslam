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

#ifndef TAGSLAM__GRAPH_UPDATER_HPP_
#define TAGSLAM__GRAPH_UPDATER_HPP_

#include <deque>
#include <memory>
#include <set>
#include <tagslam/graph.hpp>
#include <tagslam/init_pose.hpp>
#include <tagslam/profiler.hpp>
#include <tagslam/value/pose.hpp>

namespace tagslam
{
class Optimizer;
namespace value
{
class Pose;
}

struct SubGraph
{
  typedef std::deque<VertexDesc> FactorCollection;
  typedef std::set<VertexDesc> ValueCollection;
  FactorCollection factors;
  ValueCollection values;
  double error_{0};
};

class GraphUpdater
{
  using string = std::string;

public:
  GraphUpdater(const GraphUpdater &) = delete;
  GraphUpdater();
  typedef std::deque<VertexDesc> VertexDeque;
  // ---------------------
  void setOptimizerMode(const std::string & mode);
  void processNewFactors(Graph * g, uint64_t t, const VertexVec & facs);
  void printPerformance();
  double getPixelNoise() const { return (pixelNoise_); }
  const string & getOptimizerMode() const { return (optimizerMode_); }
  void parse(const YAML::Node & config);

private:
  typedef std::map<uint64_t, VertexVec> TimeToVertexesMap;
  void examine(
    Graph * graph, uint64_t t, VertexDesc fac, VertexDeque * factorsToExamine,
    SubGraph * found, SubGraph * sg);

  std::vector<VertexDeque> findSubgraphs(
    Graph * g, uint64_t t, const VertexVec & fac, SubGraph * found);

  double initializeSubgraphs(
    Graph * g, std::vector<GraphPtr> * subGraphs,
    const std::vector<VertexDeque> & verts);
  void exploreSubGraph(
    Graph * g, uint64_t t, VertexDesc start, SubGraph * subGraph,
    SubGraph * found);
  bool applyFactorsToGraph(
    Graph * g, uint64_t t, const VertexVec & facs, SubGraph * covered);
  void eraseStoredFactors(
    uint64_t t, const SubGraph::FactorCollection & covered);
  double optimize(Graph * g, double thresh);
  // ------ variables --------------
  TimeToVertexesMap oldFactors_;
  Profiler profiler_;
  int numIncrementalOpt_{0};
  double subgraphError_{0};
  double lastIncError_{0};
  bool optimizeFullGraph_{false};

  string optimizerMode_{"slow"};
  double maxSubgraphError_{15.0};
  double subGraphAbsPriorPositionNoise_{0.001};
  double subGraphAbsPriorRotationNoise_{0.001};
  double pixelNoise_{0.001};
  int maxNumIncrementalOpt_{100};
  init_pose::Params poseInitParams_;
  double minimumViewingAngle_{0};
  double maxAmbiguityRation_{0.3};
  double ambiguityAngleThreshold_{1.0};
};
}  // namespace tagslam
#endif  // TAGSLAM__GRAPH_UPDATER_HPP_