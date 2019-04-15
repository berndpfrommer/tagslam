/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/camera2.h"
#include <ros/console.h>
#include <boost/range/irange.hpp>


namespace tagslam {
  using boost::irange;
  using std::string;
  static void bombout(const string &param, const string &cam) {
    throw (std::runtime_error("cannot find " + param + " for cam " + cam));
  }

  Camera2
  Camera2::parse_camera(const string &prefix, const string &name,
                        const ros::NodeHandle &nh) {
    Camera2 cam;
    cam.name_ = name;
    string p = prefix + "/";
    cam.intrinsics_ = CameraIntrinsics2::parse(p, nh);
    if (!nh.getParam(p + "rostopic",  cam.imageTopic_)) {
      bombout("rostopic", prefix); }
    nh.param<string>(p + "tagtopic", cam.tagTopic_, "");
    if (!nh.getParam(p + "rig_body", cam.rigName_)) {
      bombout("rig_body", prefix); }
    nh.param<string>(p + "frame_id", cam.frameId_, cam.name_);
    double wiggleRot(0.00001), wiggleTrans(0.00001);
    nh.getParam(p + "wiggle_rotation", wiggleRot);
    nh.getParam(p + "wiggle_translation", wiggleTrans);
    cam.wiggle_ = PoseNoise2::make(wiggleRot, wiggleTrans);
    return (cam);
  }

  Camera2Vec
  Camera2::parse_cameras(const string &prefix, const ros::NodeHandle &nh) {
    Camera2Vec cdv;
    int cam_idx = 0;
    for (const auto cam_num: irange(0, 100)) {
      const string name = "cam" + std::to_string(cam_num);
      const string p = prefix + "/" + name;
      XmlRpc::XmlRpcValue lines;
      if (!nh.getParam(p, lines)) {
        continue;
      }
      Camera2Ptr camera(new Camera2(parse_camera(p, name, nh)));
      camera->index_ = cam_idx;
      cdv.push_back(camera);
    }
    return (cdv);
  }

}
