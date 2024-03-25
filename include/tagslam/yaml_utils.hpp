/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <string>
#include <tagslam/geometry.hpp>
#include <tagslam/pose_noise.hpp>

namespace tagslam
{
namespace yaml_utils
{
using std::string;
void write_matrix(
  std::ostream & of, const string & prefix, const Transform & pose);
void write_pose(
  std::ostream & of, const string & prefix, const Transform & pose,
  const PoseNoise & n, bool writeNoise);
void write_pose_with_covariance(
  std::ostream & of, const string & prefix, const Transform & pose,
  const PoseNoise & n);
template <typename T>
void write_container(
  std::ostream & of, const string & pf, T c, int w = 12, int p = 8)
{
  of << "[";
  for (int i = 0; i < (int)c.size() - 1; i++) {
    of << std::fixed << std::setw(w) << std::setprecision(p);
    of << c[i] << ",";
  }
  if (!c.empty()) {
    of << std::fixed << std::setw(w) << std::setprecision(p);
    of << c[c.size() - 1];
  }
  of << "]";
}
}  // namespace yaml_utils
}  // namespace tagslam
