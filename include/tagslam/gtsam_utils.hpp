// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TAGSLAM__POSE_GTSAM_UTILS_HPP_
#define TAGSLAM__POSE_GTSAM_UTILS_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <tagslam/geometry.hpp>
#include <tagslam/pose_noise.hpp>

namespace tagslam
{
namespace gtsam_utils
{
inline gtsam::Pose3 to_gtsam(const Transform & t)
{
  return (gtsam::Pose3(t.matrix()));
}
inline Transform from_gtsam(const gtsam::Pose3 & p)
{
  return (Transform(p.matrix()));
}
gtsam::noiseModel::Gaussian::shared_ptr to_gtsam(const PoseNoise & pn);
}  // namespace gtsam_utils
}  // namespace tagslam
#endif  // TAGSLAM__POSE_GTSAM_UTILS_HPP_