/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_slam2.h"
#include "tagslam/geometry.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/body_defaults.h"
#include "tagslam/body.h"

namespace tagslam {
  TagSlam2::TagSlam2() {
  }

  TagSlam2::~TagSlam2() {
  }

  void TagSlam2::onInit() {
    nh_ = getPrivateNodeHandle();
    initialize();
  }

  bool TagSlam2::initialize() {
    nh_.param<std::string>("param_prefix", paramPrefix_, "tagslam_config");
    graph_.startUpdate();
    readBodies();
    graph_.endUpdate();
    //
    optimizer_.add(&graph_.getSubGraph());
    optimizer_.optimizeFullGraph();
    return (true);
  }
  void TagSlam2::readBodies() {
    XmlRpc::XmlRpcValue config;
    nh_.getParam(paramPrefix_, config);

    // read body defaults first in case
    // in case bodies do not provide all parameters
    BodyDefaults::parse(config);

    // now read bodies
    BodyVec bv = Body::parse_bodies(config);
    for (const auto &body: bv) {
      graph_.addBody(*body);
    }
  }
}  // end of namespace


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tagslam::TagSlam2, nodelet::Nodelet)
