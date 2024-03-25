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

#ifndef TAGSLAM__POSE_NOISE_HPP_
#define TAGSLAM__POSE_NOISE_HPP_

#include <memory>
#include <tagslam/geometry.hpp>

namespace tagslam
{
class PoseNoise
{
public:
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PoseNoise(const Matrix6d & n = Matrix6d::Identity(), bool isDiag = false)
  : noise_(n), isDiagonal_(isDiag){};
  Eigen::Matrix<double, 6, 1> getDiagonal() const;
  const Matrix6d getCovarianceMatrix() const;
  bool getIsDiagonal() const { return (isDiagonal_); }
  Matrix6d convertToR() const;
  static PoseNoise make(const Point3d & angle, const Point3d & pos);
  static PoseNoise make(double a, double p);
  static PoseNoise makeFromR(const Matrix6d & r);
  friend std::ostream & operator<<(std::ostream & os, const PoseNoise & pn);

private:
  Matrix6d noise_;  // this is the covariance, *not* sigma
  bool isDiagonal_ = {false};
};
std::ostream & operator<<(std::ostream & os, const PoseNoise & pn);
typedef std::shared_ptr<PoseNoise> PoseNoisePtr;
typedef std::shared_ptr<const PoseNoise> PoseNoiseConstPtr;
}  // namespace tagslam

#endif  // TAGSLAM__POSE_NOISE_HPP_