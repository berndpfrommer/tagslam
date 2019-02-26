/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include "tagslam/optimizer.h"
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>

namespace tagslam {
  class GTSAMOptimizer: public Optimizer {
  public:
    GTSAMOptimizer();
    ~GTSAMOptimizer();
    // ---- implement Optimizer interface
    void      optimize() override;
    void      optimizeFullGraph() override;
    Transform getPose(ValueKey key) override;
    ValueKey  addPose(const Transform &pose) override;
    FactorKey addRelativePosePrior(ValueKey key1, ValueKey key2,
                                   const PoseWithNoise &deltaPose) override;
    FactorKey addAbsolutePosePrior(ValueKey key,
                                   const PoseWithNoise &pose) override;
    gtsam::ExpressionFactorGraph  &getGraph() { return (newGraph_); }
    
  private:
    inline ValueKey generateKey() { return (++key_); } // starts at 1!
    // ------------ variables ------------
    ValueKey                      key_{0};
    gtsam::Values                 values_;
    gtsam::Values                 newValues_;
    std::shared_ptr<gtsam::ISAM2> isam2_;
    gtsam::ExpressionFactorGraph  fullGraph_;
    gtsam::ExpressionFactorGraph  newGraph_;
  };
}
