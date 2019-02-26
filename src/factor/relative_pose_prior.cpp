/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/factor/relative_pose_prior.h"
#include "tagslam/value/value.h"
#include "tagslam/optimizer.h"
#include <memory>
#include <sstream>

namespace tagslam {
  namespace factor {
    std::string RelativePosePrior::getLabel() const {
      std::stringstream ss;
      ss << "rpp:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }

    // ---- methods for optimizer

    static void get_values(const value::Value **v1,
                           const value::Value **v2,
                           const BoostGraph::vertex_descriptor &f,
                           const BoostGraph &g) {
      typedef BoostGraph::out_edge_iterator OutEdgeIterator;
      OutEdgeIterator it, itEnd;
      std::tie(it, itEnd) = boost::out_edges(f, g);
      if (it == itEnd) {
        throw std::runtime_error("has 0 edges: " + g[f].vertex->getLabel());
      }
      BoostGraph::vertex_descriptor targ_1(boost::target(*it, g));
      *v1 = dynamic_cast<const value::Value *>(g[targ_1].vertex.get());
      if (! *v1) {
        throw std::runtime_error("edge 0 no val: " + g[f].vertex->getLabel());
      }
      if (++it == itEnd) {
        throw std::runtime_error("only 1 edge: " + g[f].vertex->getLabel());
      }
      BoostGraph::vertex_descriptor targ_2(boost::target(*it, g));
      *v2 = dynamic_cast<const value::Value *>(g[targ_2].vertex.get());
      if (! *v2) {
        throw std::runtime_error("edge 1 no val: " + g[f].vertex->getLabel());
      }
      if (++it != itEnd) {
        throw std::runtime_error("has > 2 edges: " + g[f].vertex->getLabel());
      }
    }

    void
    RelativePosePrior::addToOptimizer(Optimizer *opt,
                                      const BoostGraph::vertex_descriptor &v,
                                      const BoostGraph *g) {
      const value::Value *v1, *v2;
      get_values(&v1, &v2, v, *g);
      ValueKey key1 = v1->getKey();
      ValueKey key2 = v2->getKey();
      opt->addRelativePosePrior(key1, key2, poseWithNoise_);
    }

  }
}  // namespace
