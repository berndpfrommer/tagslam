/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <sstream>
#include "tagslam/value/pose.h"
#include "tagslam/optimizer.h"
#include "tagslam/graph.h"

namespace tagslam {
  namespace value {
    VertexDesc Pose::attachTo(Graph *g) const {
      auto v = std::shared_ptr<value::Pose>(new value::Pose(*this));
      v->setKey(0); // clear optimizer key
      return (g->add(v));
    }
    void Pose::addToOptimizer(Graph *g) {
      g->addToOptimizer(this);
    }
    std::string Pose::getLabel() const {
      std::stringstream ss;
      ss << "p:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }
  }
}  // namespace
