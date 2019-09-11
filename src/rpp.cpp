/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/logging.h"
#include "tagslam/rpp.h"
#include "tagslam/quartic.h"
#include <boost/range/irange.hpp>
#include <iostream>
#include <math.h>

//
// Implementation of tests for checking if tag can be 'flipped',
// and a second valid pose can be obtained. This gives a measure of
// how well the tag-to-camera transform is established.
//
// See paper: "Robust Pose Estimation from a Planar Target"
// by Gerald Schweighofer and Alex Pinz
//
// Notes:
//
// 1) The paper is a bit nebulous about how to compute \tilde{R}_z^-1.
//    What it means concretely: decompose
//
//    \tilde{R}_1 = Rz(gamma) * Ry(beta) * Rz(gamma'),
//
//    and set \tilde{R}_z^{-1} = Rz(gamma'). Then 
//
//    \tilde{R}_1 * \tilde{R}_z = Rz(gamma) * Ry(beta),
//
//    i.e. only z and y rotations.
//    
// 2) top of page 6, beta_t = tan(1/(2*beta)) is wrong, it should
//    read beta_t = tan(beta/2)
//
// 3) I cannot reproduce their expression for the gradient. I think it's
//    wrong, but not sure. I derived my own, which works. This affects
//    equations (12) and (13) in the paper:
//
//    (12) becomes:
//
//    t_opt = -G * sum_i (I-\tilde{V}_i)^2 R_z(gamma) R_y(beta) \tilde{p}_i
//
//    with G = (sum_i (I-\tilde{V}_i)^2)^{-1}
//
//    (13) is modified accordingly:
//
//    E_os = sum_i |... - G sum_j(I -\tilde{V}_j)^2 R_z  R_y \tilde{p}_i|^2

// #define DEBUG

namespace tagslam {
  namespace rpp {
    using boost::irange;

    static double eval_poly(double x, const double *a, int n) {
      double p = 1.0;
      double sum = 0;
      for (const auto i: irange(0, n)) {
        sum += p * a[i];
        p = p * x;
      }
      return (sum);
    }

    //
    // computes rotation R_t from the paper: it rotates the optical axis
    // to face to the origin (center) of the tag.
    //
    static Transform rotate_to_z(
      const Eigen::Vector3d &translat, double *ang) {
      const double t_norm = translat.norm();
      const Eigen::Vector3d txz = translat.cross(Eigen::Vector3d::UnitZ());
      const double sin_a_t_norm = txz.norm();
      if (std::abs(sin_a_t_norm) < 1e-8) {
        return Eigen::Isometry3d::Identity();
      }
      const double sin_a = sin_a_t_norm / t_norm;
      const Eigen::Vector3d n = txz / sin_a_t_norm;
      //const Transform tf = Eigen::AngleAxisd(std::asin(sin_a), n);
      *ang = std::asin(sin_a);
      const Transform tf = (Transform) Eigen::AngleAxisd(*ang, n);
      return (tf);
    }

    typedef std::vector<Eigen::Matrix3d,
                        Eigen::aligned_allocator<Eigen::Matrix3d> > M33dVec;

    //
    // make normalized matrices \tilde{V}_i from \tilde{v}_i
    //
    static M33dVec V_from_v(const ImgPointsH &v) {
      M33dVec V(v.rows());
      for (int i = 0; i < v.rows(); i++) {
        const double vnsq = v.row(i).squaredNorm();
        if (vnsq > 1e-12) {
          V[i] = v.row(i).transpose()*v.row(i) / vnsq;
        } else {
          V[i] = Eigen::Matrix3d::Zero();
        }
      }
      return (V);
    }

    //
    // computes G from the paper (but using my formula)
    //
    static Eigen::Matrix3d compute_G(const M33dVec &V_tilde) {
      Eigen::Matrix3d ImV_sum = Eigen::Matrix3d::Zero();
      for (const auto &V_i: V_tilde) {
        const auto ImV = Eigen::Matrix3d::Identity() - V_i;
        ImV_sum = ImV_sum + ImV.transpose() * ImV;
      }
      return (ImV_sum.inverse());
    }

    //
    // little helper matrix K that extracts beta_t to various powers
    // from R_y(beta_t)*p
    //
    static Eigen::Matrix3d make_K(const Eigen::Vector3d &p) {
      Eigen::Matrix3d K;
      K <<
        p(0),  2*p(2), -p(0),
        p(1),     0.0,  p(1),
        p(2), -2*p(0), -p(2);
      return (K);
    }

    //
    // compute helper matrix FTF = F^T * F.
    //
    // The error E can be expressed then as:
    //
    // E_os(beta_t) = (1+beta_t^2)^{-2} * mu^T * (F^T * F) * mu
    //
    // where mu = [1, beta_t, beta_t^2]^T
    //

    static Eigen::Matrix3d compute_FTF(const M33dVec &V_tilde,
                                       const Transform &R_z,
                                       const ObjPoints &p_tilde) {
      const int n = p_tilde.rows(); // number of points
      const auto G = compute_G(V_tilde);
      Eigen::Matrix3d C_sum = Eigen::Matrix3d::Zero();
      M33dVec C(n);
      for (const auto i: irange(0ul, V_tilde.size())) {
        const auto ImV_i = Eigen::Matrix3d::Identity() - V_tilde[i];
        C[i] = ImV_i * R_z * make_K(p_tilde.row(i));
        C_sum = C_sum + ImV_i.transpose() * C[i];
      }
      Eigen::Matrix3d FTF = Eigen::Matrix3d::Zero();
      for (const auto i: irange(0ul, V_tilde.size())) {
        const auto ImV_i = Eigen::Matrix3d::Identity() - V_tilde[i];
        const auto F_i   = C[i] - ImV_i * G * C_sum;
        FTF = FTF + F_i.transpose() * F_i;
      }
      return (FTF);
    }

    //
    // Starting from
    //
    // E_os(beta_t) = (1+beta_t^2)^{-2} * mu^T * (F^T * F) * mu
    // where mu = [1, beta_t, beta_t^2]^T
    //
    // now express E_os(beta_t) and derivates as polynomials in beta_t
    //
    // E_os   = (1+beta^2)^{-2} * (sum_{i=0^n} f[i] beta^i)
    // E_os'  = (1+beta^2)^{-3} * (sum_{i=0^n} g[i] beta^i)
    // E_os'' = (1+beta^2)^{-4} * (sum_{i=0^n} h[i] beta^i)
    //
    //
    static void compute_polynomial(const Eigen::Matrix3d &FTF,
                                   double *f, double *g, double *h) {
      // polynomial coefficients f[0] == zeroth order etc
      //
      // E_os = (f[0] + f[1] * beta_t + ... f[4] * beta_t^4) / (1+beta_t^2)^2
      //
      f[0] = FTF(0, 0);
      f[1] = FTF(0, 1) + FTF(1, 0);
      f[2] = FTF(0, 2) + FTF(1, 1) + FTF(2, 0);
      f[3] = FTF(1, 2) + FTF(2, 1);
      f[4] = FTF(2, 2);

      // first derivate:
      g[4] = -f[3];
      g[3] = 4 * f[4] - 2 * f[2];
      g[2] = 3 * f[3] - 3 * f[1];
      g[1] = 2 * f[2] - 4 * f[0];
      g[0] = f[1];

      // second derivative:
      h[0] =  -4*f[0] +  2*f[2];
      h[1] = -12*f[1] +  6*f[3];
      h[2] =  20*f[0] - 16*f[2] + 12 * f[4];
      h[3] =  12*f[1] - 16*f[3];
      h[4] =   6*f[2] - 12*f[4];
      h[5] =   2*f[3];
    }

    //
    // Finds locations beta_t of real minima and value E there
    // Returns number of minima found.
    //

    static int find_minima(
      const double *f, // poly coeff for E_os
      const double *g, // poly coeff first deriv
      const double *h, // poly coeff second deriv
      double *beta_min, double *beta_max, double *E_min, double *E_max) { 
      std::complex<double> root[4];
      // find roots of first derivative
      const int nroots = quartic::solve_quartic(g[4],g[3],g[2],g[1],g[0],root);
      
      int n_min(0);
      *E_min = 1e90;
      *E_max = -1e90;
      // check all real roots, and evaluate second deriv there
      for (const auto i:irange(0, nroots)) {
        if (std::imag(root[i]) < 1e-8) {
          const double beta_t = std::real(root[i]);
          if (eval_poly(beta_t, h, 6) > 0) {
            const double opbs = (1.0 + beta_t * beta_t);
            const double E_os = eval_poly(beta_t, f, 5)/(opbs * opbs);
            if (E_os < *E_min) {
              *E_min = E_os;
              *beta_min = 2.0 * std::atan(beta_t);
            }
            if (E_os > *E_max) {
              *E_max = E_os;
              *beta_max = 2.0 * std::atan(beta_t);
            }
            n_min++;
          }
        }
      }
      return (n_min);
    }

    //
    // computes ratio of error for lowest/(second lowest) minimum error
    // orientation. The lower the ratio, the better is the pose
    // established, the more robust it is to flipping.
    //
    double check_quality(const ImgPoints &ip, const ObjPoints &op,
                         const Transform &T, double *beta_orig,
                         double *beta_min, double *beta_max) {
      const ImgPointsH iph = ip.rowwise().homogeneous();
      // compute R_t, the matrix that rotates z to the optical axis
      double ang;
      const Transform R_t = rotate_to_z(T.translation(), &ang);
      const auto R1_tilde = R_t * T;
      // decompose R1_tilde = Rz * Ry * Rz0
      const auto   euler_angles = R1_tilde.rotation().eulerAngles(2, 1, 2);
      const double gamma  = euler_angles[0]; // z  rotation
      const double beta   = euler_angles[1]; // y  rotation
      *beta_orig = beta;
      const double alpha  = euler_angles[2]; // z0 rotation
      const Transform R_z = (Transform)
        Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ());
      const Transform R_z0 = (Transform)
        Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitZ());
      // compute v_tilde from equation (5)
      const ImgPointsH v_tilde = (R_t.rotation()*iph.transpose()).transpose();
      // compute p_tilde
      const ObjPoints p_tilde  = (R_z0.rotation()*op.transpose()).transpose();
      M33dVec V_tilde = V_from_v(v_tilde);
      const Eigen::Matrix3d FTF = compute_FTF(V_tilde, R_z, p_tilde);
      double f[5], g[5], h[6];
      compute_polynomial(FTF, f, g, h);
      double E_min, E_max;
      int n_min = find_minima(f, g, h, beta_min, beta_max, &E_min, &E_max);
      switch (n_min) {
      case 2:
        return (E_min / E_max); // two minima, the usual case
        break;
      case 1:
        return (0.0); // single minimum, assume all is good
        break;
      default:
        ROS_WARN_STREAM("found bad num minima: " << n_min);
        *beta_min = beta;
        *beta_max = beta;
        return (0.0); // close eyes and hope for the best....
        break;
      }
      return (0.0); // should never reach this
    }
  } // end of namespace rpp
}
