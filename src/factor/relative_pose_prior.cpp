/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/factor/relative_pose_prior.h"
#include "tagslam/value/value.h"
#include "tagslam/optimizer.h"
#include <memory>
#include <sstream>

namespace tagslam {
  namespace factor {
    std::string RelativePosePrior::getLabel() const {
      std::stringstream ss;
      ss << "rpp:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }
  }
}  // namespace
