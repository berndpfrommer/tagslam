/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/vertex.h"

namespace tagslam {
  namespace factor {
    class Factor: public Vertex {
    public:
      virtual std::string getLabel() const override { return (name); }
      Factor(const std::string &s = ""): Vertex(s, "box") {}
    };
  }
}
