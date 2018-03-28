/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_POSE_ESTIMATE_H
#define TAGSLAM_POSE_ESTIMATE_H

#include "tagslam/pose_noise.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <iostream>

namespace tagslam {
  class PoseEstimate: public gtsam::Pose3 {
  public:
    PoseEstimate(const gtsam::Pose3 &p = gtsam::Pose3(),
                 double e = 1e10,
                 int nit = 1000,
                 const PoseNoise &n = PoseNoise()) :
      gtsam::Pose3(p), noise(n), err(e), numIter(nit) {}
    friend std::ostream &operator<<(std::ostream &os, const PoseEstimate &pe);
    bool         isValid() const { return (err < 1e10); }
    void         setValid(bool b) { err = b ? 0.0 : 1e10; }
    void         setError(double e) { err = e; }
    double       getError() const { return (err); }
    PoseNoise    getNoise() const { return (noise); }
    gtsam::Pose3 getPose() const { return (*this); }
  private:
    PoseNoise noise;
    double    err{1e10};
    int       numIter{1000};
  };
  std::ostream &operator<<(std::ostream &os, const PoseEstimate &pe);
}

#endif
