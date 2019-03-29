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
    VertexDesc RelativePosePrior::attachTo(Graph *g) const {
      auto f = std::shared_ptr<RelativePosePrior>(new RelativePosePrior(*this));
      f->clearKeys();
      return (g->add(f));
    }
    void RelativePosePrior::addToOptimizer(Graph *g) {
      g->addToOptimizer(this);
    }
    std::string RelativePosePrior::getLabel() const {
      std::stringstream ss;
      ss << "rpp:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }
  }
}  // namespace
