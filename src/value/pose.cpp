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
#include <tagslam/optimizer.hpp>
#include <tagslam/value/pose.hpp>

namespace tagslam
{
namespace value
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("pose")); }

VertexDesc Pose::addToGraph(const VertexPtr & vp, Graph * g) const
{
  return (g->insertVertex(vp));
}

void Pose::addToOptimizer(const Transform & tf, Graph * g) const
{
  LOG_DEBUG("adding pose to opt: " << *this);
  const VertexDesc v = g->find(this);
  checkIfValid(v, "pose not found");
  g->verifyUnoptimized(v);
  const ValueKey vk = g->getOptimizer()->addPose(tf);
  g->markAsOptimized(v, vk);
}

std::string Pose::getLabel() const
{
  std::stringstream ss;
  ss << "p:" << name_ << ",t:" << format_time(time_);
  return (ss.str());
}
}  // namespace value
}  // namespace tagslam
