/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/vertex.h"

namespace tagslam {
  namespace value {
    class Value: public Vertex {
    public:
      Value(const std::string &s = ""): Vertex(s, "ellipse") {}
      virtual std::string getLabel() const override {
        return (Vertex::getLabel());
      }
    };
  }
}
