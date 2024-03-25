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

#include <iostream>
#include <tagslam/geometry.hpp>
#include <tagslam/logging.hpp>

namespace tagslam
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("geometry")); }

Point3d make_point(const std::vector<double> & vec)
{
  if (vec.size() != 3) {
    BOMB_OUT("3d point has only " << vec.size() << "components!");
  }
  return (Point3d(vec[0], vec[1], vec[2]));
}

Transform make_transform(const Eigen::Matrix3d & rot, const Point3d & trans)
{
  Transform tf = Transform::Identity();
  tf.linear() = rot;
  tf.translation() = trans;
  return (tf);
}

Transform make_transform(const Point3d & rvec, const Point3d & trans)
{
  Eigen::Matrix<double, 3, 1> axis{1.0, 0, 0};
  double angle(0);
  const double n = rvec.norm();
  if (n > 1e-8) {
    axis = rvec / n;
    angle = n;
  }
  Eigen::Matrix<double, 3, 3> rot(Eigen::AngleAxis<double>(angle, axis));
  return (make_transform(rot, trans));
}

Transform make_transform(const Eigen::Quaterniond & q, const Point3d & trans)
{
  return (make_transform(q.toRotationMatrix(), trans));
}

std::ostream & operator<<(std::ostream & os, const Transform & tf)
{
  os << "[" << tf.linear() << "]," << std::endl
     << "[" << tf.translation().transpose() << "]";
  return (os);
}
}  // namespace tagslam
