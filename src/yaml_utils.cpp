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

#include <Eigen/Geometry>
#include <cmath>
#include <tagslam/yaml_utils.hpp>

namespace tagslam
{
using std::fixed;
using std::setprecision;
using std::setw;
using std::sqrt;

#define FMT(X, Y) fixed << setw(X) << setprecision(Y)

namespace yaml_utils
{
static void write_vec(
  std::ostream & of, const string & prefix, double x, double y, double z)
{
  const int p(8);
  of.precision(p);
  of << prefix << "x: " << std::fixed << x << std::endl;
  of << prefix << "y: " << std::fixed << y << std::endl;
  of << prefix << "z: " << std::fixed << z << std::endl;
}

void write_pose(
  std::ostream & of, const string & prefix, const Transform & pose,
  const PoseNoise & n, bool writeNoise)
{
  Eigen::AngleAxisd aa(pose.linear());
  Point3d r = aa.angle() * aa.axis();
  Point3d t(pose.translation());
  const string pps = prefix + "  ";
  of << prefix << "position:" << std::endl;
  write_vec(of, pps, t(0), t(1), t(2));
  of << prefix << "rotation:" << std::endl;
  write_vec(of, pps, r(0), r(1), r(2));
  if (writeNoise) {
    Eigen::Matrix<double, 6, 1> diag = n.getDiagonal();
    of << prefix << "position_noise:" << std::endl;
    write_vec(of, pps, sqrt(diag(3)), sqrt(diag(4)), sqrt(diag(5)));
    of << prefix << "rotation_noise:" << std::endl;
    write_vec(of, pps, sqrt(diag(0)), sqrt(diag(1)), sqrt(diag(2)));
  }
}

void write_matrix(
  std::ostream & of, const string & prefix, const Transform & pose)
{
  const auto m = pose.matrix();
  for (int i = 0; i < 4; i++) {
    of << prefix << "- [";
    for (int j = 0; i < 3; i++) {
      of << FMT(12, 8) << m(i, j) << ",";
    }
    of << FMT(12, 8) << m(i, 3) << "]" << std::endl;
  }
}

void write_pose_with_covariance(
  std::ostream & of, const string & prefix, const Transform & pose,
  const PoseNoise & n)
{
  Eigen::AngleAxisd aa;
  aa.fromRotationMatrix(pose.rotation());
  Eigen::Vector3d r = aa.angle() * aa.axis();
  Eigen::Vector3d t = pose.translation();
  const string pps = prefix + "  ";
  of << prefix << "position:" << std::endl;
  write_vec(of, pps, t(0), t(1), t(2));
  of << prefix << "rotation:" << std::endl;
  write_vec(of, pps, r(0), r(1), r(2));
  of << prefix << "R:" << std::endl;
  const auto R = n.convertToR();
  of << prefix << "  [ ";
  for (int i = 0; i < R.rows(); i++) {
    for (int j = 0; j < R.cols(); j++) {
      of << R(i, j);
      if (i != R.rows() - 1 || j != R.cols() - 1) {
        of << ", ";
      }
    }
    if (i != R.rows() - 1) {
      of << std::endl << prefix << "    ";
    }
  }
  of << " ]" << std::endl;
}
}  // namespace yaml_utils
}  // namespace tagslam
