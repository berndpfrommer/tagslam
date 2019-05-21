/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/factor/absolute_pose_prior.h"
#include "tagslam/value/value.h"
#include "tagslam/optimizer.h"
#include "tagslam/graph.h"
#include <sstream>

namespace tagslam {
  namespace factor {
    VertexDesc
    AbsolutePosePrior::addToGraph(const VertexPtr &vp, Graph *g) const {
      // NOTE: prior name and pose name must match!
      const VertexDesc cp = g->findPose(getTime(), vp->getName());
      checkIfValid(cp, "no pose for absolute pose prior");
      const VertexDesc fv = g->insertFactor(vp);
      g->addEdge(fv, cp, 0);
      return (fv);
    }

    void AbsolutePosePrior::addToOptimizer(Graph *g) const {
      const VertexDesc v = g->find(this);
      checkIfValid(v, "factor not found");
      g->verifyUnoptimized(v);
      const std::vector<ValueKey> optKeys = g->getOptKeysForFactor(v, 1);
      const FactorKey fk = g->getOptimizer()->addAbsolutePosePrior(
        optKeys[0], getPoseWithNoise());
      g->markAsOptimized(v, fk);
    }

    string AbsolutePosePrior::getLabel() const {
      std::stringstream ss;
      ss << "app:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }
  } // namespace factor
}  // namespace tagslam
