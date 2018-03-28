/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/pose_estimate.h"

namespace tagslam {
  std::ostream &operator<<(std::ostream &os, const PoseEstimate &pe) {
    os << "pose: " << pe.getPose() << " noise: " << pe.getNoise() << " err: " << pe.getError();
    return (os);
  }
}  // namespace
