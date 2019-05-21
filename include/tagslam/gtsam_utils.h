/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/geometry.h"
#include "tagslam/pose_noise.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

namespace tagslam {
  namespace gtsam_utils {
    inline gtsam::Pose3 to_gtsam(const Transform &t) {
      return (gtsam::Pose3(t.matrix()));
    }
    inline Transform from_gtsam(const gtsam::Pose3 &p) {
      return (Transform(p.matrix()));
    }
    boost::shared_ptr<gtsam::noiseModel::Gaussian>
    to_gtsam(const PoseNoise &pn);
  }
}
