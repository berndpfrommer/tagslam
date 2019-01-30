/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/factor/absolute_pose_prior.h"
#include "tagslam/value/value.h"
#include "tagslam/gtsam_optimizer.h"
#include "tagslam/gtsam_utils.h"
#include <gtsam/slam/PriorFactor.h>
#include <sstream>

namespace tagslam {
  namespace factor {
    std::string AbsolutePosePrior::getLabel() const {
      std::stringstream ss;
      ss << "app:" << name << ",t:" << time.toSec();
      return (ss.str());
    }

    // ---- methods for optimizer adding
    void
    AbsolutePosePrior::addToOptimizer(GTSAMOptimizer *opt,
                                      const BoostGraph::vertex_descriptor &v,
                                      const BoostGraph *g) {
      typedef BoostGraph::out_edge_iterator OutEdgeIterator;
      OutEdgeIterator it, itEnd;
      std::tie(it, itEnd) = boost::out_edges(v, *g);
      if (it == itEnd) {
        std::cout << "ERROR: AbsolutePosePrior has no target!" << std::endl;
        return;
      }
      BoostGraph::vertex_descriptor src(boost::source(*it, *g)),
        targ(boost::target(*it, *g));
      const std::shared_ptr<Vertex> vtx = (*g)[targ].vertex;
      if (!vtx->isValue()) {
        std::cout << "ERROR: AbsolutePosePrior has invalid value!" << std::endl;
        return;
      }
      const value::Value *val = dynamic_cast<value::Value const *>(vtx.get());
      if (val == NULL) {
        std::cout << "ERROR: AbsolutePosePrior has invalid value type!" << std::endl;
        return;
      }
      ValueKey key = val->getKey();
      auto &graph = opt->getNewGraph();
      graph.push_back(gtsam::PriorFactor<gtsam::Pose3>
                      (key, gtsam_utils::to_gtsam(poseWithNoise.getPose()),
                       gtsam_utils::to_gtsam(poseWithNoise.getNoise())));
      if (++it != itEnd) {
        std::cout << "ERROR: AbsolutePosePrior " << (*g)[targ].vertex->getLabel()
                  << " has too many edges!" << std::endl;
        throw (std::runtime_error("too many edges: " + (*g)[targ].vertex->getLabel()));
      }
    }
  } // namespace factor
}  // namespace tagslam
