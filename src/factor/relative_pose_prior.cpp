/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/factor/relative_pose_prior.h"
#include "tagslam/value/value.h"
#include "tagslam/optimizer.h"
#include "tagslam/graph.h"
#include <memory>
#include <sstream>

namespace tagslam {
  namespace factor {
    VertexDesc
    RelativePosePrior::addToGraph(const VertexPtr &vp, Graph *g) const {
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

    void RelativePosePrior::addToOptimizer(Graph *g) const {
      const VertexDesc v = g->find(this);
      checkIfValid(v, "factor not found");
      g->verifyUnoptimized(v);
      const std::vector<ValueKey> optKeys = g->getOptKeysForFactor(v, 2);
      const FactorKey fk = g->getOptimizer()->addRelativePosePrior(
        optKeys[0], optKeys[1], getPoseWithNoise());
      g->markAsOptimized(v, fk);
    }

    string RelativePosePrior::getLabel() const {
      std::stringstream ss;
      ss << "rpp:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }
  }
}  // namespace
