/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <Eigen/Geometry>
#include <vector>

namespace tagslam {
  typedef Eigen::Vector2d Point2d;
  typedef Eigen::Vector3d Point3d;
  typedef Eigen::Isometry3d Transform;
  Transform make_transform(const Eigen::Quaterniond &q, const Point3d &trans);
  Transform make_transform(const Eigen::Matrix3d &rot,  const Point3d &trans);
  Transform make_transform(const Point3d   &rvec, const Point3d &trans);
  Point3d make_point(const std::vector<double> &vec);
  std::ostream &operator<<(std::ostream &os, const Transform &tf);
}
