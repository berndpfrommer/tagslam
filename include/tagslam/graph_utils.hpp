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

#ifndef TAGSLAM__GRAPH_UTILS_HPP_
#define TAGSLAM__GRAPH_UTILS_HPP__

#include <string>
#include <tagslam/body.hpp>
#include <tagslam/camera.hpp>
#include <tagslam/graph.hpp>
#include <tagslam/pose_with_noise.hpp>
#include <tagslam/tag.hpp>

namespace tagslam
{
namespace graph_utils
{
using std::string;
// convenience functions for adding to graph
void add_pose_maybe_with_prior(
  Graph * g, uint64_t t, const string & name, const PoseWithNoise & pwn,
  bool isCamPose);
void add_tag(Graph * g, const Tag & tag);
void add_body(Graph * g, const Body & body);

// Copy all vertices specified in srcfacs from the src graph
// to the destination graph. Also copy all values that are used by
// those factors, and pin them down with an absolute pose prior to
// be their current values.
void copy_subgraph(
  Graph * dest, const Graph & src, const std::deque<VertexDesc> & srcfacs,
  double absPriorPositionNoise, double absPriorRotationNoise);
// Look through the source graph for any values that are not yet
// optimized in the destination graph. Initialize those values
// in the destination graph, and add them to the optimizer.
// Also add any destination graph factors to the optimizer if
// they can now be optimized
void initialize_from(Graph * destg, const Graph & srcg);

// Removes from graph all factors and values listed. Optimized values
// are kept! Caller must make sure that the graph is not
// ill determined afterwards!
void filter_graph(
  Graph * g, const std::set<VertexDesc> & factorsToRemove,
  const std::set<VertexDesc> & valuesToRemove);

// convenience functions for retrieval of optimized poses
bool get_optimized_pose(
  const Graph & g, uint64_t t, const string & name, Transform * tf);
bool get_optimized_pose(const Graph & g, const Camera & cam, Transform * tf);
bool get_optimized_pose(
  const Graph & g, uint64_t t, const Body & body, Transform * tf);
bool get_optimized_pose(const Graph & g, const Tag & tag, Transform * tf);

PoseWithNoise get_optimized_pose_with_noise(
  const Graph & g, uint64_t t, const string & name);
// graph plotting
void plot(const string & fname, const Graph * g);
void plot_debug(uint64_t t, const string & tag, const Graph & g);
}  // namespace graph_utils
}  // namespace tagslam

#endif  // TAGSLAM__GRAPH_UTILS_HPP__