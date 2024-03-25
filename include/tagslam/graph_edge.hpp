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

#ifndef TAGSLAM__GRAPH_EDGE_HPP_
#define TAGSLAM__GRAPH_EDGE_HPP_

#include <iostream>

namespace tagslam
{
struct GraphEdge
{
  GraphEdge(int i = 0) : edge_property(i){};
  int edge_property;
  friend std::ostream & operator<<(std::ostream & os, const GraphEdge & e);
};
std::ostream & operator<<(std::ostream & os, const GraphEdge & e);
}  // namespace tagslam

#endif  // TAGSLAM__GRAPH_EDGE_HPP_
