/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include "tagslam/graph.h"
#include "tagslam/value_key.h"
#include "tagslam/factor_key.h"
#include "tagslam/geometry.h"

namespace tagslam {
  class Optimizer {
  public:
    Optimizer() {}
    virtual ~Optimizer() {};

    virtual void optimize() = 0;
    virtual void optimizeFullGraph() = 0;


    // retrieves the optimized pose for a given key
    virtual Transform getPose(ValueKey key) = 0;
    // adds the starting guess for a new value (e.g. camera pose)
    virtual ValueKey addPose(const Transform &pose) = 0;
    // relative pose prior, i.e. err = ||Pose(key1) - deltaPose * Pose(key2)||
    virtual FactorKey addRelativePosePrior(ValueKey key1, ValueKey key2,
                                           const PoseWithNoise &deltaPose) = 0;
    // absolute pose prior, i.e. err = ||Pose(key) - pose||
    virtual FactorKey addAbsolutePosePrior(ValueKey key,
                                           const PoseWithNoise &pose) = 0;
  private:
  };
}
