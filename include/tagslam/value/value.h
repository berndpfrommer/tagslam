/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/vertex.h"
#include "tagslam/value_key.h"

namespace tagslam {
  namespace value {
    class Value: public Vertex {
    public:
      Value(const std::string &s = ""): Vertex(s, "ellipse") {}
      bool isValue() const override { return (true); }
      virtual std::string getLabel() const override {
        return (Vertex::getLabel());
      }
      ValueKey getKey() const { return (key_); }
      void setKey(ValueKey k) { key_ = k; }
    private:
      ValueKey key_{0}; // zero is invalid!
    };
  }
}
