/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <sstream>
#include "tagslam/factor/relative_pose_prior.h"

namespace tagslam {
  namespace factor {
    std::string RelativePosePrior::getLabel() const {
      std::stringstream ss;
      ss << "rpp:" << name << ",t:" << time.toSec();
      return (ss.str());
    }
  }
}  // namespace
