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

#ifndef TAGSLAM__CAMERA_HPP_
#define TAGSLAM__CAMERA_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <tagslam/camera_intrinsics.hpp>
#include <tagslam/pose_with_noise.hpp>

namespace tagslam
{
class Body;  // need forward declaration here
class Camera
{
  using string = std::string;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // --------- typedefs -------
  typedef std::shared_ptr<Camera> CameraPtr;
  typedef std::shared_ptr<const Camera> CameraConstPtr;
  typedef std::vector<CameraPtr> CameraVec;
  // --- getters/setters
  const string & getName() const { return (name_); }
  const string & getImageTopic() const { return (imageTopic_); }
  const string & getTagTopic() const { return (tagTopic_); }
  const string & getFrameId() const { return (frameId_); }
  const string & getRigName() const { return (rigName_); }
  const std::shared_ptr<Body> getRig() const { return (rig_); }
  const PoseNoise & getWiggle() const { return (wiggle_); }
  const CameraIntrinsics & getIntrinsics() const { return (intrinsics_); }
  int getIndex() const { return (index_); }
  void setRig(const std::shared_ptr<Body> & rig) { rig_ = rig; }

  // --- static methods
  static CameraPtr parse_camera(const string & name, const YAML::Node & config);
  static CameraVec parse_cameras(const YAML::Node & config);

private:
  // -------- variables -------
  string name_;
  int index_{-1};
  string imageTopic_;            // topic for images
  string tagTopic_;              // topic for tags
  string frameId_;               // ros frame id of camera
  string rigName_;               // name of rig body
  std::shared_ptr<Body> rig_;    // pointer to rig body
  CameraIntrinsics intrinsics_;  // intrinsic calibration
  PoseNoise wiggle_;             // how rigid the ext calib is
};

using CameraPtr = Camera::CameraPtr;
using CameraConstPtr = Camera::CameraConstPtr;
using CameraVec = Camera::CameraVec;
}  // namespace tagslam
#endif  // TAGSLAM__CAMERA_INTRINSICS_HPP_