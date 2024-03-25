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

#include <tagslam/geometry.hpp>
#include <tagslam/rpp.hpp>

//
// just a short driver to cut down on link
// times while debugging
//

int main(int argc, char ** argv)
{
  Eigen::Matrix<double, 4, 3> wp;
  wp << -1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, -1.0, 0.0, -1.0, -1.0, 0.0;
  Eigen::Matrix<double, 4, 2> ip;
  ip << -0.10543, 0.1491, 0.1336, 0.18893, 0.1336, -0.18893, -0.10543, -0.1491;

  const tagslam::Transform T =
    Eigen::Translation3d(Eigen::Vector3d::UnitZ() * 6.0) *
    Eigen::AngleAxisd(3.14159265 / 180.0 * 45.0, Eigen::Vector3d::UnitY());

  //std::cout << T.matrix() << std::endl;
  double beta_orig, beta_min, beta_max;
  const double tau =
    tagslam::rpp::check_quality(ip, wp, T, &beta_orig, &beta_min, &beta_max);
  std::cout << "quality ratio: " << tau << " " << std::endl;
  return (0);
}
