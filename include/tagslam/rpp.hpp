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

#pragma once

#include <Eigen/Dense>
#include <tagslam/geometry.hpp>

namespace tagslam
{
namespace rpp
{
typedef Eigen::Matrix<double, 4, 2> ImgPoints;
typedef Eigen::Matrix<double, 4, 3> ImgPointsH;
typedef Eigen::Matrix<double, 4, 3> ObjPoints;
typedef Eigen::Matrix<double, 4, 4> ObjPointsH;
//
// computes ratio of error for lowest/(second lowest) minimum error
// orientation. The lower the ratio, the better is the pose
// established, the more robust it is to flipping.
//
double check_quality(
  const ImgPoints & ip, const ObjPoints & op, const Transform & T,
  double * beta_orig, double * beta_min, double * beta_max);
}  // namespace rpp
}  // namespace tagslam
