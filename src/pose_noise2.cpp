/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/pose_noise2.h"

#include <iostream>

namespace tagslam {
  // static function
  PoseNoise2 PoseNoise2::make(const Point3d &a, const Point3d &p) {
    Matrix6d m = Matrix6d::Zero();
    m.diagonal() << a(0),a(1),a(2),p(0),p(1),p(2);
    return (PoseNoise2(m, true));
  }

  // static function
  PoseNoise2 PoseNoise2::make(double a, double p) {
    return (make(Point3d(a, a, a), Point3d(p, p, p)));
  }
 
  static const Matrix6d sqrt_info_to_sigma(const Matrix6d &R) {
    // TODO: test this! Is it working at all???
    // Could be just as well R * R.transpose()
    const Matrix6d rsqi = (R.transpose() * R).inverse();
    return (rsqi);
  }

  // static method
  PoseNoise2 PoseNoise2::makeFromR(const Matrix6d &R) {
    const Matrix6d sigma = sqrt_info_to_sigma(R);
    return (PoseNoise2(sigma, false /*isdiag*/));
  }

  Eigen::Matrix<double, 6, 1> PoseNoise2::getDiagonal() const {
    return (noise.diagonal());
  }

  std::ostream &operator<<(std::ostream &os, const PoseNoise2 &pn) {
    os << "is_diagonal: " << pn.isDiagonal << std::endl << pn.noise;
    return (os);
  }
    
}  // namespace
