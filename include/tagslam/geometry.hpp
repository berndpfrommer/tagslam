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

#ifndef TAGSLAM__GEOMETRY_HPP_
#define TAGSLAM__GEOMETRY_HPP_

#include <Eigen/Geometry>
#include <vector>

namespace tagslam
{
typedef Eigen::Vector2d Point2d;
typedef Eigen::Vector3d Point3d;
typedef Eigen::Isometry3d Transform;
Transform make_transform(const Eigen::Quaterniond & q, const Point3d & trans);
Transform make_transform(const Eigen::Matrix3d & rot, const Point3d & trans);
Transform make_transform(const Point3d & rvec, const Point3d & trans);
Point3d make_point(const std::vector<double> & vec);
std::ostream & operator<<(std::ostream & os, const Transform & tf);
}  // namespace tagslam
#endif  // TAGSLAM__GEOMETRY_HPP_