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

#ifndef TAGSLAM__CAMERA_INTRINSICS_HPP_
#define TAGSLAM__CAMERA_INTRINSICS_HPP_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <string>
#include <tagslam/distortion_model.hpp>
#include <vector>

namespace YAML
{
class Node;  // foward decl;
}

namespace tagslam
{
class CameraIntrinsics
{
  using string = std::string;

public:
  friend std::ostream & operator<<(
    std::ostream & os, const CameraIntrinsics & ci);
  const std::vector<double> & getDVec() const { return (distortionCoeffs_); }
  const std::vector<double> & getKVec() const { return (K_); }
  const cv::Mat & getK() const { return (cvK_); }
  const cv::Mat & getD() const { return (cvD_); }
  const DistortionModel & getDistortionModel() const
  {
    return (distortionModel_);
  }
  // i/o functions
  void writeYaml(std::ostream & f, const string & pf) const;
  // static functions
  static CameraIntrinsics parse(const YAML::Node & config);

private:
  static CameraIntrinsics parse_no_error(const YAML::Node & config);
  std::vector<double> distortionCoeffs_;
  std::vector<double> K_;  // K Matrix
  std::vector<int> resolution_;
  DistortionModel distortionModel_;
  string cameraModel_;
  cv::Mat cvK_;  // precomputed for speed
  cv::Mat cvD_;  // precomputed for speed
};
std::ostream & operator<<(std::ostream & os, const CameraIntrinsics & ci);
}  // namespace tagslam

#endif  // TAGSLAM__CAMERA_INTRINSICS_HPP_