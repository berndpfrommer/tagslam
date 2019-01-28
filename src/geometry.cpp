/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/geometry.h"

namespace tagslam {
  Transform make_transform(const Eigen::Matrix3d &rot, const Point3d &trans) {
    Transform tf;
    tf.linear()      = rot;
    tf.translation() = trans;
    return (tf);
  }

  Transform make_transform(const Point3d &rvec, const Point3d &trans) {
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

}  // namespace
