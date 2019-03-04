/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/vertex.h"
#include "tagslam/factor_key.h"

namespace tagslam {
  namespace factor {
    class Factor: public Vertex {
    public:
      virtual std::string getLabel() const override { return (name_); }
      bool isValue() const override { return (false); }
      Factor(const std::string &s = ""): Vertex(s, "box") {}
      FactorKey getKey()  const { return (key_); }
      void     setKey(FactorKey k) { key_ = k; }
      bool     isOptimized() const { return (key_ != 0); }
    private:
      FactorKey key_{0}; // zero is invalid!
    };
  }
  typedef std::shared_ptr<factor::Factor> FactorPtr;
  typedef std::shared_ptr<const factor::Factor> FactorConstPtr;
}
