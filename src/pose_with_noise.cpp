/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/pose_with_noise.h"

namespace tagslam {
  std::ostream &operator<<(std::ostream &os, const PoseWithNoise &pe) {
    //os << "pose: " << pe.getPose() << " noise: " << pe.getNoise().getDiagonal().transpose();
    os << "poSe: " << " noise: " << pe.getNoise().getDiagonal().transpose();
    return (os);
  }
}  // namespace
