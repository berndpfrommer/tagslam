/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/geometry.h"

namespace tagslam {
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  class PoseNoise2 {
  public:
    PoseNoise2(const Matrix6d &n = Matrix6d::Identity()) :
      noise(n) {
    };
    static PoseNoise2 make(const Point3d &angle,  const Point3d &pos);
    static PoseNoise2 make(double a, double p);
    Eigen::Matrix<double, 6, 1> getDiagonal() const;
  private:
    Matrix6d noise;
  };
}
