/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/pose_noise.h"

namespace tagslam {
  PoseNoise makePoseNoise(const Eigen::Vector3d &a,
                          const Eigen::Vector3d &p) {
    gtsam::Vector sn(6);
    sn << a(0),a(1),a(2),p(0),p(1),p(2);
    return (gtsam::noiseModel::Diagonal::Sigmas(sn));
  }
  PoseNoise makePoseNoise(double angle, double position) {
    gtsam::Vector sn(6);
    sn << angle,angle,angle,position,position,position;
    return (gtsam::noiseModel::Diagonal::Sigmas(sn));
  }
}  // namespace
