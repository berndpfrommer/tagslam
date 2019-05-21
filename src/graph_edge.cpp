/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */


#include "tagslam/graph_edge.h"
namespace tagslam {
  std::ostream &operator<<(std::ostream &os, const GraphEdge &e) {
    os << e.edge_property;
    return (os);
  }
}
