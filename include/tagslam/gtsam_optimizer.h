/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include "tagslam/graph.h"
#include "tagslam/value_key.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>

namespace tagslam {
  class GTSAMOptimizer {
  public:
    GTSAMOptimizer();
    ~GTSAMOptimizer();
    // --- typedefs and methods that are common to all optimizers
    //
    // TODO: move to separate interface

    void     add(BoostGraph *g);
    ValueKey generateKey() { return (++key_); } // starts at 1!
    void     optimize();
    void     optimizeFullGraph();
  
    // ---- methods specific to GTSAM -----

    // adds the starting guess for a new value (e.g. camera pose)
    void addValue(ValueKey key, const gtsam::Pose3 &v);
    gtsam::ExpressionFactorGraph &getNewGraph() { return (newGraph_); }

  private:
    ValueKey      key_{0};
    gtsam::Values values_;
    gtsam::Values newValues_;
    std::shared_ptr<gtsam::ISAM2> graph_;
    gtsam::ExpressionFactorGraph  fullGraph_;
    gtsam::ExpressionFactorGraph  newGraph_;
  };
}
