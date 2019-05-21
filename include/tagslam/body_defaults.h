/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <memory>
#include <ros/ros.h>

namespace tagslam {
  struct BodyDefaults {
    BodyDefaults(double p, double r) :
      positionNoise(p), rotationNoise(r) {}
    static std::shared_ptr<BodyDefaults> instance();
    static void parse(XmlRpc::XmlRpcValue &config);
    double positionNoise{1e-1};
    double rotationNoise{1e-1};
  };
}
