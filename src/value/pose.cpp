/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <sstream>
#include "tagslam/value/pose.h"

namespace tagslam {
  namespace value {
    std::string Pose::getLabel() const {
      std::stringstream ss;
      ss << "p:" << name << ",t:" << time.toSec();
      return (ss.str());
    }
  }
}  // namespace
