/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/geometry.h"

namespace tagslam {
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  class PoseNoise2 {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseNoise2(const Matrix6d &n = Matrix6d::Identity(),
               bool isDiag = false) :
      noise(n), isDiagonal(isDiag) {
    };
    Eigen::Matrix<double,6,1>  getDiagonal() const;
    const Matrix6d            &getSigmaMatrix() const { return (noise); }
    bool                       getIsDiagonal() const { return (isDiagonal); }
    static PoseNoise2 make(const Point3d &angle,  const Point3d &pos);
    static PoseNoise2 make(double a, double p);
  private:
    Matrix6d noise;
    bool     isDiagonal = {false};
  };
}
