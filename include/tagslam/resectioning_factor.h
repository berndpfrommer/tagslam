/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 * Adapted from GTSAM resectioning example.
 */
/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
#ifndef TAGSLAM_RESECTIONING_FACTOR_H
#define TAGSLAM_RESECTIONING_FACTOR_H

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <boost/make_shared.hpp>

/**
 * Unary factor on the unknown pose, resulting from measuring
 * the projection of  a known 3D point in the image.
 */
namespace tagslam {
  class ResectioningFactor: public gtsam::NoiseModelFactor1<gtsam::Pose3> {
  public:
    /// Construct factor given known point P and its projection p
    ResectioningFactor(const gtsam::SharedNoiseModel& model, const gtsam::Key& key,
                       const boost::shared_ptr<gtsam::Cal3DS2>& calib, const gtsam::Point2& p,
                       const gtsam::Point3& P) :
      Base(model, key), K_(calib), P_(P), p_(p) {
    }
    /// evaluate the error
    virtual gtsam::Vector
    evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix&> H =
                  boost::none) const {
      gtsam::PinholeCamera<gtsam::Cal3DS2> camera(pose, *K_);
      gtsam::Point2 reprojectionError(camera.project(P_, H) - p_);
      return reprojectionError.vector();
    }
  private:
    typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;
    boost::shared_ptr<gtsam::Cal3DS2> K_; ///< camera's intrinsic parameters
    gtsam::Point3 P_;              ///< 3D point on the calibration rig
    gtsam::Point2 p_;              ///< 2D measurement of the 3D point
  };
} // namespace

#endif
