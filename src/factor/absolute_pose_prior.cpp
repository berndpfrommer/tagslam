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
#include <tagslam/factor/absolute_pose_prior.hpp>
#include <tagslam/graph.hpp>
#include <tagslam/optimizer.hpp>
#include <tagslam/value/value.hpp>

namespace tagslam
{
namespace factor
{
VertexDesc AbsolutePosePrior::addToGraph(const VertexPtr & vp, Graph * g) const
{
  // NOTE: prior name and pose name must match!
  const VertexDesc cp = g->findPose(getTime(), vp->getName());
  checkIfValid(cp, "no pose for absolute pose prior");
  const VertexDesc fv = g->insertFactor(vp);
  g->addEdge(fv, cp, 0);
  return (fv);
}

void AbsolutePosePrior::addToOptimizer(Graph * g) const
{
  const VertexDesc v = g->find(this);
  checkIfValid(v, "factor not found");
  g->verifyUnoptimized(v);
  const std::vector<ValueKey> optKeys = g->getOptKeysForFactor(v, 1);
  const FactorKey fk =
    g->getOptimizer()->addAbsolutePosePrior(optKeys[0], getPoseWithNoise());
  g->markAsOptimized(v, fk);
}

string AbsolutePosePrior::getLabel() const
{
  std::stringstream ss;
  ss << "app:" << name_ << ",t:" << format_time(time_);
  return (ss.str());
}
}  // namespace factor
}  // namespace tagslam
