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

#ifndef TAGSLAM__INIT_POSE_HPP_
#define TAGSLAM__INIT_POSE_HPP_

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <string>
#include <tagslam/distortion_model.hpp>
#include <tagslam/geometry.hpp>
#include <utility>

namespace tagslam
{
namespace init_pose
{
struct Params
{
  double minViewingAngle;
  double ambiguityAngleThreshold;
  double maxAmbiguityRatio;
};
std::pair<Transform, bool> pose_from_4(
  const Eigen::Matrix<double, 4, 2> & imgPoints,
  const Eigen::Matrix<double, 4, 3> & objPoints, const cv::Mat & K,
  DistortionModel distModel, const cv::Mat & D, const Params & params);
std::string cv_type_to_str(int type);
std::string cv_info(const cv::Mat & m);
std::string print_as_python_mat(const cv::Mat & m);
}  // namespace init_pose
}  // namespace tagslam

#endif  // TAGSLAM__INIT_POSE_HPP_