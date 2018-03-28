/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_POSE_NOISE_H
#define TAGSLAM_POSE_NOISE_H

#include <gtsam/linear/NoiseModel.h>
#include <Eigen/Dense>

namespace tagslam {
  using PoseNoise = gtsam::noiseModel::Diagonal::shared_ptr;
  PoseNoise makePoseNoise(const Eigen::Vector3d &a,
                                 const Eigen::Vector3d &p);
  PoseNoise makePoseNoise(double angle, double position);
}

#endif
