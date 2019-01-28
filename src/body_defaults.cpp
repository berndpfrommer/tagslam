/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/body_defaults.h"

#include <XmlRpcException.h>

namespace tagslam {
  std::shared_ptr<BodyDefaults> s_ptr;

  std::shared_ptr<BodyDefaults> BodyDefaults::instance() {
    return (s_ptr);
  }
  void BodyDefaults::parse(XmlRpc::XmlRpcValue &config) {
    try {
      XmlRpc::XmlRpcValue def = config["body_defaults"];
      double pn = static_cast<double>(def["position_noise"]);
      double rn = static_cast<double>(def["rotation_noise"]);
      s_ptr.reset(new BodyDefaults(pn, rn));
    } catch (const XmlRpc::XmlRpcException &e) {
      throw std::runtime_error("error parsing body defaults!");
    }

  }

} // end of namespace
