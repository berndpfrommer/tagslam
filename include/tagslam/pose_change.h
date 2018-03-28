/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_POSE_CHANGE_H
#define TAGSLAM_POSE_CHANGE_H

#include <gtsam/geometry/Pose3.h>

namespace tagslam {
  struct PoseChange {
    PoseChange(double r=0, double t = 0) :
      rot(r), trans(t) {
    }
    double  rot;
    double  trans;
    // -----------
    static PoseChange pose_change(const gtsam::Pose3 &p1, const gtsam::Pose3 &p2);
  };
}

#endif
