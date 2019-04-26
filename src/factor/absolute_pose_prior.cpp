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
    VertexDesc AbsolutePosePrior::attach(const VertexPtr &vp, Graph *g) const {
      AbsolutePosePriorFactorPtr fp =
        std::dynamic_pointer_cast<factor::AbsolutePosePrior>(vp);
      return (g->add(fp));
    }
    void AbsolutePosePrior::addToOptimizer(Graph *g) const {
      g->addToOptimizer(this);
    }
    std::string AbsolutePosePrior::getLabel() const {
      std::stringstream ss;
      ss << "app:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }
  } // namespace factor
}  // namespace tagslam
