/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/geometry.h"
#include <Eigen/Dense>


namespace tagslam {
  namespace quartic {
    typedef std::complex<double> Complex;
    int solve_linear(const double a, const double b, Complex *root);
    int solve_quadratic(const double a, const double b, const double c,
                        Complex *root);
    int solve_cubic(const double a, const double b, const double c,
                    const double d, Complex *root);

    int solve_quartic(const double a, const double b, const double c,
                      const double d, const double e,
                      Complex *root);
  }
}
