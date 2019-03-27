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
  typedef std::shared_ptr<Vertex> GraphVertex;
  std::ostream &operator<<(std::ostream &os, const GraphVertex &v);

}
