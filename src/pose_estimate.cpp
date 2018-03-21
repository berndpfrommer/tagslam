/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/pose_estimate.h"

namespace tagslam {
  PoseEstimate::PoseChange
  PoseEstimate::pose_change(const gtsam::Pose3 &p1,
                            const gtsam::Pose3 &p2) {
    gtsam::Pose3 errPose = p1.between(p2);
    gtsam::Vector3 er = gtsam::Rot3::Logmap(errPose.rotation());
    gtsam::Vector3 et = errPose.translation();
    return (PoseEstimate::PoseChange(
              std::sqrt(er(0)*er(0) + er(1)*er(1) + er(2)*er(2)),
              std::sqrt(et(0)*et(0) + et(1)*et(1) + et(2)*et(2))));
  }
}  // namespace
