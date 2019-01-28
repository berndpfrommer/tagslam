/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */


#include "tagslam/graph_vertex.h"
namespace tagslam {
  std::ostream &operator<<(std::ostream &os, const GraphVertex &v) {
    os << v.vertex->getLabel();
    return (os);
  }
}
