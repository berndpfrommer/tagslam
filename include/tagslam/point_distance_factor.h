/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_POINT_DISTANCE_FACTOR_H
#define TAGSLAM_POINT_DISTANCE_FACTOR_H

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <boost/make_shared.hpp>
#include <iostream>

/**
 * Binary factor to express distance measurement between two points
 */
namespace tagslam {
  class PointDistanceFactor: public gtsam::NoiseModelFactor2<gtsam::Point3,
                                                             gtsam::Point3> {
  public:
    PointDistanceFactor(const gtsam::SharedNoiseModel& model, const gtsam::Key& key1,
                        const gtsam::Key& key2, double d) :
      Base(model, key1, key2), distance_(d) {
    }
    /// evaluate the error
    virtual gtsam::Vector
    evaluateError(const gtsam::Point3 &p1, const gtsam::Point3 &p2, boost::optional<gtsam::
                  Matrix&> H1 =
                  boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none) const override {
      gtsam::Vector ve(1);
      double len = p2.distance(p1);
      ve(0) = len - distance_;
      if (H1 || H2) {
        double linv = (len < 1e-8)? 0 :1.0/len;
        gtsam::Matrix13 nvec = (p2-p1) * linv;
        if (H1) {
          *H1 = -nvec;
        }
        if (H2) {
          *H2 = nvec;
        }
      }
      return (ve);
    }
  private:
    typedef gtsam::NoiseModelFactor2<gtsam::Point3, gtsam::Point3> Base;
    double distance_; // measured distance between the two points
  };
} // namespace

#endif
