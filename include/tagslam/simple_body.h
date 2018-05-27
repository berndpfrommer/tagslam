/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_SIMPLE_BODY_H
#define TAGSLAM_SIMPLE_BODY_H

#include "tagslam/rigid_body.h"
#include <iostream>

namespace tagslam {
  struct SimpleBody: public RigidBody {
    SimpleBody(const std::string &n  = std::string(""),
               bool iS = false) : RigidBody(n, iS) {
      type = "simple";
    }
    typedef std::shared_ptr<SimpleBody>       SimpleBodyPtr;
    typedef std::shared_ptr<const SimpleBody> SimpleBodyConstPtr;

    bool parse(XmlRpc::XmlRpcValue body_defaults,
               XmlRpc::XmlRpcValue body) override;
    bool write(std::ostream &os, const std::string &prefix) const override;
  };
  using SimpleBodyPtr      = SimpleBody::SimpleBodyPtr;
  using SimpleBodyConstPtr = SimpleBody::SimpleBodyConstPtr;
}

#endif
