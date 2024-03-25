/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
// gtsam's Cal3DS2 class is missing the uncalibrate() method

#ifndef TAGSLAM_CAL3DS2U_H
#define TAGSLAM_CAL3DS2U_H
#include <gtsam/geometry/Cal3DS2.h>

class Cal3DS2U : public gtsam::Cal3DS2 {
public:
  Cal3DS2U(double fx, double fy, double s, double u0, double v0,
           double k1, double k2, double p1 = 0.0, double p2 = 0.0) :
    gtsam::Cal3DS2(fx, fy, s, u0, v0, k1, k2, p1, p2) {}
  Cal3DS2U(const gtsam::Cal3DS2 &cal) : Cal3DS2(cal) {
  }

  gtsam::Point2 uncalibrate(
    const gtsam::Point2& p, gtsam::OptionalJacobian<2,9> Dcal = boost::none,
    gtsam::OptionalJacobian<2,2> Dp = boost::none) const {
    return (gtsam::Cal3DS2_Base::uncalibrate(p, Dcal, Dp));
  }
};
// now it gets even uglier, as I had to put this into the gtsam namespace
namespace gtsam {
  template<> struct traits<Cal3DS2U> : public internal::Manifold<Cal3DS2> {};
  template<> struct traits<const Cal3DS2U> : public internal::Manifold<Cal3DS2> {};
}

#endif
