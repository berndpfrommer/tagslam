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

#pragma once
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <tagslam/graph_edge.hpp>
#include <tagslam/graph_vertex.hpp>

namespace tagslam
{
typedef boost::adjacency_list<
  boost::listS, boost::vecS, boost::undirectedS, GraphVertex, GraphEdge>
  BoostGraph;
typedef BoostGraph::vertex_descriptor BoostGraphVertex;
}  // namespace tagslam
