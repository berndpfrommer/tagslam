/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <sstream>
#include "tagslam/value/pose.h"
#include "tagslam/optimizer.h"
#include "tagslam/graph.h"

namespace tagslam {
  namespace value {
    VertexDesc Pose::attach(const VertexPtr &vp, Graph *g) const {
      PoseValuePtr pp = std::dynamic_pointer_cast<value::Pose>(vp);
      return (g->add(pp));
    }
    std::string Pose::getLabel() const {
      std::stringstream ss;
      ss << "p:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }
  }
}  // namespace
