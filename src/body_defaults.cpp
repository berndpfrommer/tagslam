/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/body_defaults.h"
#include "tagslam/xml.h"
#include "tagslam/logging.h"

#include <XmlRpcException.h>

namespace tagslam {
  std::shared_ptr<BodyDefaults> s_ptr;

  std::shared_ptr<BodyDefaults> BodyDefaults::instance() {
    return (s_ptr);
  }
  void BodyDefaults::parse(XmlRpc::XmlRpcValue &config) {
    if (!config.hasMember("body_defaults")) {
      BOMB_OUT("no body defaults found!");
    }
    try {
      XmlRpc::XmlRpcValue def = config["body_defaults"];
      const double pn = xml::parse<double>(def, "position_noise");
      const double rn = xml::parse<double>(def, "rotation_noise");
      s_ptr.reset(new BodyDefaults(pn, rn));
    } catch (const XmlRpc::XmlRpcException &e) {
      BOMB_OUT("error parsing body defaults!");
    }
  }

} // end of namespace
