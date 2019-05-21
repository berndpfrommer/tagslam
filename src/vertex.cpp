/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */


#include "tagslam/vertex.h"
#include "tagslam/graph.h"
#include "tagslam/logging.h"

#include <sstream>

namespace tagslam {
  std::ostream &operator<<(std::ostream &os, const Vertex &v) {
    os << v.toString();
    return (os);
  }

  std::string Vertex::toString() const {
    return (getLabel());
  }

  std::string Vertex::format_time(const ros::Time &t) {
    // wrap around every 100 seconds,
    // resolution is in milliseconds
    uint64_t tw = (t.toNSec() % 1000000000000UL) / 1000000L;
    std::stringstream ss;
    ss << tw;
    return (ss.str());
  }

  void Vertex::checkIfValid(const VertexDesc &v, const std::string &m) const {
    if (!Graph::is_valid(v)) {
      BOMB_OUT("invalid: " << m << ": " << getLabel());
    }
  }

}
