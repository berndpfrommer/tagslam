/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/camera.h"
#include "tagslam/xml.h"
#include "tagslam/logging.h"

#include <ros/console.h>
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>

namespace tagslam {
  using boost::irange;
  using std::string;

  CameraPtr
  Camera::parse_camera(const string &name, XmlRpc::XmlRpcValue config) {
    CameraPtr camPtr(new Camera());
    Camera &cam = *camPtr; // short hand
    cam.name_ = name;
    cam.intrinsics_ = CameraIntrinsics::parse(config);
    cam.imageTopic_ = xml::parse<string>(config, "rostopic");
    cam.tagTopic_   = xml::parse<string>(config, "tagtopic", "");
    cam.rigName_    = xml::parse<string>(config, "rig_body");
    cam.frameId_    = xml::parse<string>(config, "frameId", cam.name_);
    double wiggleR  = xml::parse<double>(config, "wiggle_rotation",   0.00001);
    double wiggleT  = xml::parse<double>(config, "wiggle_translation",0.00001);
    cam.wiggle_    = PoseNoise::make(wiggleR, wiggleT);
    return (camPtr);
  }

  CameraVec
  Camera::parse_cameras(XmlRpc::XmlRpcValue config) {
    CameraVec cdv;
    int cam_idx = 0;
    for (const auto cam_num: irange(0, 100)) {
      string name = "cam" + std::to_string(cam_num);
      if (!config.hasMember(name)) {
        name = "camera_" + std::to_string(cam_num);
        if (!config.hasMember(name)) {
        continue;
        }
      }
      try {
        CameraPtr camera = parse_camera(name, config[name]);
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
