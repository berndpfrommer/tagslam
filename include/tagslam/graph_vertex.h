/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

//#include "tagslam/vertex.h"
#include <string>
#include <memory>
#include <iostream>

namespace tagslam {
  class Vertex;
  class GraphVertex {
  public:
    GraphVertex(const std::shared_ptr<Vertex> &v = std::shared_ptr<Vertex>()): vertex(v) {}
    friend std::ostream &operator<<(std::ostream &os, const GraphVertex &v);

    std::shared_ptr<Vertex> vertex;
  };
  std::ostream &operator<<(std::ostream &os, const GraphVertex &v);

}
