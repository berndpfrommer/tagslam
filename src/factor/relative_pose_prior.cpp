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

#include <memory>
#include <sstream>
#include <tagslam/factor/relative_pose_prior.hpp>
#include <tagslam/graph.hpp>
#include <tagslam/optimizer.hpp>
#include <tagslam/value/value.hpp>

namespace tagslam
{
namespace factor
{
VertexDesc RelativePosePrior::addToGraph(const VertexPtr & vp, Graph * g) const
{
  // NOTE: poses and pose prior factor names must match!
  const VertexDesc pp = g->findPose(getPreviousTime(), getName());
  checkIfValid(pp, "no prev pose for relative pose prior");
  const VertexDesc cp = g->findPose(getTime(), getName());
  checkIfValid(cp, "no current pose for relative pose prior");
  const VertexDesc fv = g->insertFactor(vp);
  g->addEdge(fv, pp, 0);
  g->addEdge(fv, cp, 1);
  return (fv);
}

void RelativePosePrior::addToOptimizer(Graph * g) const
{
  const VertexDesc v = g->find(this);
  checkIfValid(v, "factor not found");
  g->verifyUnoptimized(v);
  const std::vector<ValueKey> optKeys = g->getOptKeysForFactor(v, 2);
  const FactorKey fk = g->getOptimizer()->addRelativePosePrior(
    optKeys[0], optKeys[1], getPoseWithNoise());
  g->markAsOptimized(v, fk);
}

string RelativePosePrior::getLabel() const
{
  std::stringstream ss;
  ss << "rpp:" << name_ << ",t:" << format_time(time_);
  return (ss.str());
}
}  // namespace factor
}  // namespace tagslam
