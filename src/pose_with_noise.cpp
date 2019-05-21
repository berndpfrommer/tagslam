/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/pose_with_noise.h"
#include "tagslam/logging.h"

namespace tagslam {
  using std::string;
  static void bombout(const string &param, const string &name) {
    BOMB_OUT("error with " << param << " for " << name);
  }

  static std::vector<double>
  make_default_noise(double sigma_rot, double sigma_trans) {
    std::vector<double> n(36, 0.0);
    n[0]  = n[7]   = n [14] = 1.0/sigma_rot;
    n[21] = n[28]  = n[35]  = 1.0/sigma_trans;
    return (n);
  }

  std::ostream &operator<<(std::ostream &os, const PoseWithNoise &pe) {
    os << "pose: valid: " << pe.isValid() << std::endl <<
      pe.getPose() << std::endl << "noise: " << pe.getNoise();
    //os << "pose: " <<pe." noise: "<<pe.getNoise().getDiagonal().transpose();
    return (os);
  }
}  // namespace
