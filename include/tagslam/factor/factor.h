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
      bool isValue() const override { return (false); }
      Factor(const std::string &s = ""): Vertex(s, "box") {}
    };
  }
}
