/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/pose_with_noise.h"

namespace tagslam {
  using std::string;
  std::ostream &operator<<(std::ostream &os, const PoseWithNoise &pe) {
    os << "pose: valid: " << pe.isValid() << std::endl <<
      pe.getPose() << std::endl << "noise: " << pe.getNoise();
    return (os);
  }
}  // namespace
