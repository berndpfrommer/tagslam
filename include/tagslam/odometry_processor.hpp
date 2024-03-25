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

#ifndef TAGSLAM__ODOMETRY_PROCESSOR_HPP_
#define TAGSLAM__ODOMETRY_PROCESSOR_HPP_

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tagslam/body.hpp>
#include <tagslam/geometry.hpp>
#include <tagslam/graph.hpp>
#include <tagslam/pose_noise.hpp>

namespace tagslam
{
class OdometryProcessor
{
  using Odometry = nav_msgs::msg::Odometry;
  using OdometryConstPtr = Odometry::ConstSharedPtr;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OdometryProcessor(rclcpp::Node * node, const BodyConstPtr & body);
  void process(
    uint64_t t, Graph * g, const OdometryConstPtr & msgs,
    std::vector<VertexDesc> * factors);
  static VertexDesc add_body_pose_delta(
    Graph * graph, uint64_t tPrev, uint64_t tCurr, const BodyConstPtr & body,
    const PoseWithNoise & deltaPose);
  void finalize() const;

private:
  PoseNoiseConstPtr makeAdaptiveNoise(uint64_t t, const Transform & deltaPose);
  void updateStatistics(uint64_t t, const Transform & d);
  // ---- variables
  BodyConstPtr body_;
  Transform pose_;
  uint64_t time_{0};
  rclcpp::Publisher<Odometry> pub_;
  Transform T_body_odom_;
  double accelerationNoiseMin_{5.0};          // m/s^2
  double angularAccelerationNoiseMin_{5.0};   // rad/sec^2
  double accelerationNoiseMax_{50.0};         // m/s^2
  double angularAccelerationNoiseMax_{50.0};  // rad/sec^2
  double translationNoise_{-1.0};             // m
  double rotationNoise_{-1.0};                // rads
  Eigen::Vector3d lastOmega_;
  Eigen::Vector3d lastVelocity_;
  double lenSum_{0};
  double len2Sum_{0};
  double angSum_{0};
  double ang2Sum_{0};
  unsigned int count_{0};
  double lenMax_{0};
  uint64_t lenMaxT_{0};
};
}  // namespace tagslam
#endif  // TAGSLAM__ODOMETRY_PROCESSOR_HPP_