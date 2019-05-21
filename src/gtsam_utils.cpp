/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <tagslam/gtsam_utils.h>

namespace tagslam {
  namespace gtsam_utils {
    boost::shared_ptr<gtsam::noiseModel::Gaussian>
    to_gtsam(const PoseNoise &pn) {
      if (pn.getIsDiagonal()) {
        return (gtsam::noiseModel::Diagonal::Sigmas(pn.getDiagonal()));
      }
      return (gtsam::noiseModel::Gaussian::Covariance(pn.getSigmaMatrix()));
    }
  }
}
