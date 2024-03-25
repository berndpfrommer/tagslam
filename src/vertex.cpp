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

#include <sstream>
#include <tagslam/graph.hpp>
#include <tagslam/logging.hpp>
#include <tagslam/vertex.hpp>

namespace tagslam
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("vertex")); }

std::ostream & operator<<(std::ostream & os, const Vertex & v)
{
  os << v.toString();
  return (os);
}

std::string Vertex::toString() const { return (getLabel()); }

std::string Vertex::format_time(uint64_t t)
{
  // wrap around every 100 seconds,
  // resolution is in milliseconds
  uint64_t tw = (t % 1000000000000UL) / 1000000L;
  std::stringstream ss;
  ss << tw;
  return (ss.str());
}

void Vertex::checkIfValid(const VertexDesc & v, const std::string & m) const
{
  if (!Graph::is_valid(v)) {
    BOMB_OUT("invalid: " << m << ": " << getLabel());
  }
}

}  // namespace tagslam
