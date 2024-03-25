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

#include <tagslam/geometry.hpp>
#include <tagslam/pose_noise.hpp>

namespace tagslam
{
class PoseWithNoise
{
public:
  using string = std::string;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PoseWithNoise(
    const Transform & tf = Transform::Identity(),
    const PoseNoise & n = PoseNoise(), bool isVal = false)
  : pose_(tf), noise_(n), valid_(isVal){};
  const Transform & getPose() const { return (pose_); }
  const PoseNoise & getNoise() const { return (noise_); }
  bool isValid() const { return (valid_); }

  void setNoise(const PoseNoise & pn) { noise_ = pn; }

private:
  Transform pose_;
  PoseNoise noise_;
  bool valid_{false};
};
std::ostream & operator<<(std::ostream & os, const PoseWithNoise & pe);
}  // namespace tagslam
