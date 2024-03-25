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

#include <climits>
#include <map>
#include <memory>
#include <tagslam/boost_graph.hpp>
#include <tagslam/factor/absolute_pose_prior.hpp>
#include <tagslam/factor/coordinate.hpp>
#include <tagslam/factor/distance.hpp>
#include <tagslam/factor/relative_pose_prior.hpp>
#include <tagslam/factor/tag_projection.hpp>
#include <tagslam/geometry.hpp>
#include <tagslam/optimizer.hpp>
#include <tagslam/pose_with_noise.hpp>
#include <tagslam/value/pose.hpp>
#include <tagslam/vertex.hpp>
#include <tagslam/vertex_desc.hpp>
#include <unordered_map>

#ifndef TAGSLAM__GRAPH_HPP_
#define TAGSLAM__GRAPH_HPP_

namespace tagslam
{
class Graph
{
  using string = std::string;

public:
  Graph();
  typedef std::multimap<double, VertexDesc> ErrorToVertexMap;
  typedef std::map<uint64_t, std::vector<std::pair<FactorConstPtr, double>>>
    TimeToErrorMap;

  bool hasId(const VertexId & id) const { return (idToVertex_.count(id) != 0); }
  bool hasPose(uint64_t t, const string & name) const;
  bool isOptimized(const VertexDesc & v) const
  {
    return (optimized_.find(v) != optimized_.end());
  }
  bool isOptimizableFactor(const VertexDesc & v) const;

  string info(const VertexDesc & v) const;
  double optimize(double thresh);
  double optimizeFull(bool force = false);

  const VertexVec & getFactors() const { return (factors_); }
  VertexVec getOptimizedFactors() const;
  std::vector<OptimizerKey> getOptimizerKeys(const VertexVec & vv) const;
  const BoostGraph & getBoostGraph() const { return (graph_); }

  VertexVec getConnected(const VertexDesc & v) const;

  void addEdge(const VertexDesc & from, const VertexDesc & to, int edgeId)
  {
    boost::add_edge(from, to, GraphEdge(edgeId), graph_);
  }

  VertexDesc addPose(uint64_t t, const string & name, bool isCamPose = false);

  Transform getOptimizedPose(const VertexDesc & v) const;
  inline Transform pose(const VertexDesc & v) const
  {
    return (getOptimizedPose(v));
  }

  PoseNoise getPoseNoise(const VertexDesc & v) const;

  PoseValueConstPtr getPoseVertex(const VertexDesc f) const
  {
    return (std::dynamic_pointer_cast<const value::Pose>(graph_[f]));
  }
  VertexPtr getVertex(const VertexDesc f) const { return (graph_[f]); }
  VertexPtr operator[](const VertexDesc f) const { return (graph_[f]); }
  std::pair<BoostGraph::vertex_iterator, BoostGraph::vertex_iterator>
  getVertexIterator() const
  {
    return (boost::vertices(graph_));
  }

  void setVerbosity(const string & v) { optimizer_->setVerbosity(v); }
  void print(const string & pre = "") const;
  string getStats() const;
  //
  // deep copy and other nasty stuff
  //
  Graph * clone() const;
  void transferFullOptimization() { optimizer_->transferFullOptimization(); }
  //
  // methods related to optimization
  //
  Optimizer * getOptimizer() const { return (optimizer_.get()); }
  std::vector<ValueKey> getOptKeysForFactor(VertexDesc fv, int nk) const;
  void markAsOptimized(const VertexDesc & v, const std::vector<FactorKey> & f);
  void markAsOptimized(const VertexDesc & v, const FactorKey & f);
  void verifyUnoptimized(const VertexDesc & v) const;
  //
  // methods for finding vertexes in the graph
  //
  VertexDesc find(const Vertex * vp) const;
  inline VertexDesc find(const VertexId & id) const
  {
    // inlined function for search by string
    const auto it = idToVertex_.find(id);
    return (it == idToVertex_.end() ? ULONG_MAX : it->second);
  }
  inline VertexDesc findPose(uint64_t t, const string & name) const
  {
    return (find(value::Pose::id(t, name)));
  }
  inline VertexDesc findTagPose(int tagId) const
  {
    return (findPose(0, tag_name(tagId)));
  }
  inline VertexDesc findBodyPose(uint64_t t, const string & n) const
  {
    return (findPose(t, body_name(n)));
  }
  inline VertexDesc findCameraPose(uint64_t t, const string & c) const
  {
    return (findPose(t, cam_name(c)));
  }
  VertexDesc insertVertex(const VertexPtr & vp);
  VertexDesc insertFactor(const VertexPtr & vp);

  // for debugging, compute error on graph
  void printUnoptimized() const;
  void printErrorMap(const string & prefix) const;
  double getError(const VertexDesc & v) const;
  double getError() { return (optimizer_->errorFull()); }
  double getMaxError() { return (optimizer_->getMaxError()); }
  void plotDebug(uint64_t t, const string & tag);
  ErrorToVertexMap getErrorMap() const;
  TimeToErrorMap getTimeToErrorMap() const;
  // static methods
  static string tag_name(int tagid);
  static string body_name(const string & body);
  static string cam_name(const string & cam);
  inline static bool is_valid(const VertexDesc & v) { return (v != ULONG_MAX); }

private:
  typedef std::unordered_map<VertexId, VertexDesc> IdToVertexMap;
  typedef std::unordered_map<VertexDesc, std::vector<OptimizerKey>>
    VertexToOptMap;
  VertexToOptMap::const_iterator findOptimized(const VertexDesc & v) const;
  ValueKey findOptimizedPoseKey(const VertexDesc & v) const;

  // ------ variables --------------
  BoostGraph graph_;
  VertexVec factors_;
  IdToVertexMap idToVertex_;
  VertexToOptMap optimized_;
  std::shared_ptr<Optimizer> optimizer_;
};
typedef std::shared_ptr<Graph> GraphPtr;
typedef std::shared_ptr<const Graph> GraphConstPtr;
}  // namespace tagslam
#endif  // TAGSLAM__GRAPH_HPP__