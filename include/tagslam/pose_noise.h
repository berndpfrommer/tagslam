/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/geometry.h"

#include <memory>

namespace tagslam {
  class PoseNoise {
  public:
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseNoise(const Matrix6d &n = Matrix6d::Identity(),
               bool isDiag = false) :
      noise_(n), isDiagonal_(isDiag) {
    };
    Eigen::Matrix<double,6,1>  getDiagonal() const;
    const Matrix6d             getCovarianceMatrix() const;
    bool                       getIsDiagonal() const { return (isDiagonal_); }
    Matrix6d                   convertToR() const;
    static PoseNoise make(const Point3d &angle,  const Point3d &pos);
    static PoseNoise make(double a, double p);
    static PoseNoise makeFromR(const Matrix6d &r);
    friend std::ostream &operator<<(std::ostream &os, const PoseNoise &pn);

  private:
    Matrix6d noise_; // this is the covariance, *not* sigma
    bool     isDiagonal_ = {false};
  };
  std::ostream &operator<<(std::ostream &os, const PoseNoise &pn);
  typedef std::shared_ptr<PoseNoise>       PoseNoisePtr;
  typedef std::shared_ptr<const PoseNoise> PoseNoiseConstPtr;
}
