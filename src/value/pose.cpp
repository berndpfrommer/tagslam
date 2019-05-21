/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <sstream>
#include "tagslam/value/pose.h"
#include "tagslam/optimizer.h"
#include "tagslam/graph.h"

namespace tagslam {
  namespace value {
    VertexDesc Pose::addToGraph(const VertexPtr &vp, Graph *g) const {
      return (g->insertVertex(vp));
    }

    void Pose::addToOptimizer(const Transform &tf, Graph *g) const {
      ROS_DEBUG_STREAM("adding pose to opt: " << *this);
      const VertexDesc v = g->find(this);
      checkIfValid(v, "pose not found");
      g->verifyUnoptimized(v);
      const ValueKey vk = g->getOptimizer()->addPose(tf);
      g->markAsOptimized(v, vk);
    }

    std::string Pose::getLabel() const {
      std::stringstream ss;
      ss << "p:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }
  }
}  // namespace
