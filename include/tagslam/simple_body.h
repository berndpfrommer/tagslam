/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/body.h"
#include <iostream>

namespace tagslam {
  struct SimpleBody: public Body {
    SimpleBody(const std::string &n  = std::string(""),
               bool iS = false) : Body(n, iS) {
      type_ = "simple";
    }
    typedef std::shared_ptr<SimpleBody>       SimpleBodyPtr;
    typedef std::shared_ptr<const SimpleBody> SimpleBodyConstPtr;

    bool parse(XmlRpc::XmlRpcValue body, const BodyPtr &bp) override;
    bool write(std::ostream &os, const std::string &prefix) const override;
  };
  using SimpleBodyPtr      = SimpleBody::SimpleBodyPtr;
  using SimpleBodyConstPtr = SimpleBody::SimpleBodyConstPtr;
}
