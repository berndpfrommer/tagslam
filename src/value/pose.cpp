/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <sstream>
#include "tagslam/value/pose.h"
#include "tagslam/gtsam_utils.h"
#include "tagslam/gtsam_optimizer.h"

namespace tagslam {
  namespace value {
    std::string Pose::getLabel() const {
      std::stringstream ss;
      ss << "p:" << name << ",t:" << time.toSec();
      return (ss.str());
    }
    void Pose::addToOptimizer(GTSAMOptimizer *opt,
                              const BoostGraph::vertex_descriptor &v,
                              const BoostGraph *g) {
      ValueKey k = opt->generateKey();
      setKey(k); // remember
      gtsam::Pose3 p = gtsam_utils::to_gtsam(poseWithNoise.getPose());
      Transform t = poseWithNoise.getPose();
      opt->addValue(k, gtsam_utils::to_gtsam(poseWithNoise.getPose()));
    };
  }
}  // namespace
