/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/camera2.h"
#include "tagslam/xml.h"
#include "tagslam/logging.h"

#include <ros/console.h>
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>

namespace tagslam {
  using boost::irange;
  using std::string;

  Camera2Ptr
  Camera2::parse_camera(const string &name, XmlRpc::XmlRpcValue config) {
    Camera2Ptr camPtr(new Camera2());
    Camera2 &cam = *camPtr; // short hand
    cam.name_ = name;
    cam.intrinsics_ = CameraIntrinsics2::parse(config);
    cam.imageTopic_ = xml::parse<string>(config, "rostopic");
    cam.tagTopic_   = xml::parse<string>(config, "tagtopic", "");
    cam.rigName_    = xml::parse<string>(config, "rig_body");
    cam.frameId_    = xml::parse<string>(config, "frameId", cam.name_);
    double wiggleR  = xml::parse<double>(config, "wiggle_rotation",   0.00001);
    double wiggleT  = xml::parse<double>(config, "wiggle_translation",0.00001);
    cam.wiggle_    = PoseNoise2::make(wiggleR, wiggleT);
    return (camPtr);
  }

  Camera2Vec
  Camera2::parse_cameras(XmlRpc::XmlRpcValue config) {
    Camera2Vec cdv;
    int cam_idx = 0;
    for (const auto cam_num: irange(0, 100)) {
      const string name = "cam" + std::to_string(cam_num);
      if (!config.hasMember(name)) {
        continue;
      }
      try {
        Camera2Ptr camera = parse_camera(name, config[name]);
        camera->index_ = cam_idx++;
        cdv.push_back(camera);
      } catch (const XmlRpc::XmlRpcException &e) {
        ROS_ERROR_STREAM("got xmlrpc exception: " <<
                         e.getCode() << " " << e.getMessage());
        BOMB_OUT("error reading camera: " << name);
      }
    }
    return (cdv);
  }

}
