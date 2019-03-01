/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <sstream>
#include "tagslam/value/pose.h"
#include "tagslam/optimizer.h"

namespace tagslam {
  namespace value {
    std::string Pose::getLabel() const {
      std::stringstream ss;
      ss << "p:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }
  }
}  // namespace
