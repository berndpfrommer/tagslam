/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_POSE_ESTIMATE_H
#define TAGSLAM_POSE_ESTIMATE_H

#include <gtsam/geometry/Pose3.h>

namespace tagslam {
  struct PoseEstimate {
    struct PoseChange {
      PoseChange(double r=0, double t = 0) :
        rot(r), trans(t) {
      }
      double  rot;
      double  trans;
    };
    gtsam::Pose3      pose;
    double            err{1e10};
    int               numIter{1000};
    PoseChange        poseChange;
    // -----------
    static PoseChange pose_change(const gtsam::Pose3 &p1, const gtsam::Pose3 &p2);
  };
}

#endif
