/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/body.h"
#include <iostream>

namespace tagslam {
  struct SimpleBody2: public Body {
    SimpleBody2(const std::string &n  = std::string(""),
               bool iS = false) : Body(n, iS) {
      type = "simple";
    }
    typedef std::shared_ptr<SimpleBody2>       SimpleBody2Ptr;
    typedef std::shared_ptr<const SimpleBody2> SimpleBody2ConstPtr;

    bool parse(XmlRpc::XmlRpcValue body) override;
    bool write(std::ostream &os, const std::string &prefix) const override;
  };
  using SimpleBody2Ptr      = SimpleBody2::SimpleBody2Ptr;
  using SimpleBody2ConstPtr = SimpleBody2::SimpleBody2ConstPtr;
}
