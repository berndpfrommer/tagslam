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
    VertexDesc AbsolutePosePrior::attachTo(Graph *g) const {
      auto f = std::shared_ptr<AbsolutePosePrior>(new AbsolutePosePrior(*this));
      f->setKey(0);
      return (g->add(f));
    }
    OptimizerKey AbsolutePosePrior::addToOptimizer(Graph *g) {
      return (g->addToOptimizer(this));
    }
    std::string AbsolutePosePrior::getLabel() const {
      std::stringstream ss;
      ss << "app:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }
  } // namespace factor
}  // namespace tagslam
