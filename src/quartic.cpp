/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/quartic.h"
#include <complex>

//
// This was implemented straight from the Wikipedia page
// on quartic functions:
//
//   https://en.wikipedia.org/wiki/Quartic_function
//
// It kinda works for the few cases that I used it, but
// I'm sure there are plenty of cases where it won't work.
//
// Test coverage:
//   - only solve_quartic() has ever been tested
//   - no tests have been done if any of the special conditions work
//   - I did verify that for some of the test cases
//     produced by TagSLAM, the roots agreed with the ones found by
//     numpy.roots().
//   - USE AT YOUR OWN RISK
//

namespace tagslam {
  namespace quartic {
    static Complex complex_cbrt(const Complex &z) {
      return (std::pow(z, 1.0/3.0));
    }
    static const double TOL(1e-12);
    static const double rad_3 = 1.7320508075688772;
    static const double two_thirds = 2.0/3.0;


    int solve_linear(const double a, const double b, Complex *root) {
      if (std::abs(a) < TOL) {
        return (0);
      }
      root[0] = Complex(-b/a);
      return (1);
    }

    int solve_quadratic(const double a, const double b, const double c,
                        Complex *root) {
      if (std::abs(a) < TOL) {
        return (solve_linear(b, c, root));
      }
      const Complex rad_D = std::sqrt(b*b - Complex(4)*a*c);
      const double a2inv = 1.0 / (2 * a);
      root[0] = a2inv * (-b + rad_D);
      root[1] = a2inv * (-b - rad_D);
      return (2);
    }
    
    int solve_cubic(const double a, const double b, const double c,
                    const double d, Complex *root) {
      if (std::abs(a) < TOL) {
        return (solve_quadratic(b, c, d, root));
      }
      const double D_0 = b*b - 3*a*c;
      const double D_1 = 2*b*b*b - 9*a*b*c + 27*a*a*d;
      
      if (std::abs(D_0) < TOL && std::abs(D_1) < TOL) {
        //
        // handle special case of C = 0
        //
        root[0] = root[1] = root[2] = -b / (3*a);
        return (3);
      }

      const Complex C_0  = complex_cbrt(
        0.5 * (D_1 + std::sqrt(D_1*D_1 - Complex(4.0)*D_0*D_0*D_0)));
      const Complex C_1  = Complex{-0.5,  0.5 * rad_3} * C_0;
      const Complex C_2  = Complex{-0.5, -0.5 * rad_3} * C_0;
      const double m_a3_inv = -1/(3.0*a);

      root[0] = m_a3_inv * (b + C_0 + D_0/C_0);
      root[1] = m_a3_inv * (b + C_1 + D_0/C_1);
      root[2] = m_a3_inv * (b + C_2 + D_0/C_2);

      return (3);
    }
    
    int solve_quartic(const double a, const double b, const double c,
                      const double d, const double e, Complex *root) {
      //
      // solve a x^4 + b x^3 + c x^2 + d x + e = 0
      //

      if (std::abs(a) < TOL) {
        return (solve_cubic(b, c, d, e, root));
      }

      const double p = (8*a*c  - 3*b*b) / (8*a*a);
      const double q = (b*b*b - 4*a*b*c + 8*a*a*d)/(8*a*a*a);
      const Complex D_0 = c*c - 3*b*d + 12*a*e;
      const Complex D_1 = 2*c*c*c - 9*b*c*d + 27*b*b*e + 27*a*d*d-72*a*c*e;
      
      const Complex Q_0 = complex_cbrt(
        (std::abs(D_0) < TOL) ? (D_1) :
        (0.5 * (D_1 + std::sqrt(D_1*D_1 - 4.0*D_0*D_0*D_0))));
      if (std::abs(Q_0) < TOL) {
        // This means D_0 = 0 and D_1 = 0. Wikipedia says this means
        // triple root + one single. Ignore this special case...
        // throw up hands and say we can't do it.
        return (0);
      }
      Complex S = 0.5 * std::sqrt(-two_thirds*p + (Q_0 + D_0/Q_0)/(3*a));
      if (std::abs(S) < TOL) {
        // try different cube root
        const Complex rot_cube = Complex{-0.5,  0.5 * rad_3};
        const Complex Q_1 = rot_cube * Q_0;
        S = 0.5 * std::sqrt(-two_thirds*p + (Q_1 + D_0/Q_1)/(3*a));
        if (std::abs(S) < TOL) {
          // try next cube root
          const Complex rot_cube_2 = Complex{-0.5, -0.5 * rad_3};
          const Complex Q_2 = rot_cube_2 * Q_0;
          S = 0.5 * std::sqrt(-two_thirds*p + (Q_2 + D_0/Q_2)/(3*a));
          if (std::abs(S) < TOL) {
            // wiki says this is quadruple root  at x = - b/(4a)
            root[0] = root[1] = root[2] = root[3] = -b/(4*a);
            return (4);
          }
        }
      }
      // We should have valid S != 0 and a !=0 at this point
      root[0] = - b/(4 * a) - S + 0.5 * std::sqrt(-4.0*S*S - 2*p + q/S);
      root[1] = - b/(4 * a) - S - 0.5 * std::sqrt(-4.0*S*S - 2*p + q/S);
      root[2] = - b/(4 * a) + S + 0.5 * std::sqrt(-4.0*S*S - 2*p - q/S);
      root[3] = - b/(4 * a) + S - 0.5 * std::sqrt(-4.0*S*S - 2*p - q/S);
      return (4); // number of valid roots
    }
  } // end of namespace quartic
}
