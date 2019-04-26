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
    VertexDesc RelativePosePrior::attach(const VertexPtr &vp, Graph *g) const {
      RelativePosePriorFactorPtr fp =
        std::dynamic_pointer_cast<factor::RelativePosePrior>(vp);
      return (g->add(fp));
    }
    void RelativePosePrior::addToOptimizer(Graph *g) const {
      g->addToOptimizer(this);
    }
    std::string RelativePosePrior::getLabel() const {
      std::stringstream ss;
      ss << "rpp:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }
  }
}  // namespace
