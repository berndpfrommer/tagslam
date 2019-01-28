/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <sstream>
#include "tagslam/factor/absolute_pose_prior.h"

namespace tagslam {
  namespace factor {
    std::string AbsolutePosePrior::getLabel() const {
      std::stringstream ss;
      ss << "app:" << name << ",t:" << time.toSec();
      return (ss.str());
    }
  }
}  // namespace
