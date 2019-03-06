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
      Factor(const std::string &s = ""): Vertex(s, "box") {}
      
      bool isValue() const override { return (false); }
      bool isOptimized() const override { return (key_ != 0); }

      FactorKey getKey()  const { return (key_); }
      void      setKey(FactorKey k) { key_ = k; }
    private:
      FactorKey key_{0};    // zero is invalid!
    };
  }
  typedef std::shared_ptr<factor::Factor> FactorPtr;
  typedef std::shared_ptr<const factor::Factor> FactorConstPtr;
}
