/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <iostream>

namespace tagslam {
  struct GraphEdge {
    GraphEdge(int i = 0) : edge_property(i) {};
    int edge_property;
    friend std::ostream &operator<<(std::ostream &os, const GraphEdge &e);
  };
  std::ostream &operator<<(std::ostream &os, const GraphEdge &e);
}
