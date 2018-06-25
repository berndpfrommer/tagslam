/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/camera.h"
#include "tagslam/pose_estimate.h"
#include "tagslam/utils.h"
#include <ros/console.h>
#include <string>
#include <sstream>
#include <vector>
#include <ctime>
#include <boost/range/irange.hpp>

using std::vector;
using std::string;

namespace tagslam {
  using boost::irange;

  static void bombout(const std::string &param, const std::string &cam) {
    throw (std::runtime_error("cannot find " + param + " for cam " + cam));
  }

  static CameraExtrinsics get_kalibr_style_transform(const ros::NodeHandle &nh,
                                                     const std::string &field) {
    CameraExtrinsics T;
    XmlRpc::XmlRpcValue lines;
    if (!nh.getParam(field, lines)) {
      throw (std::runtime_error("cannot find transform " + field));
    }
    if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw (std::runtime_error("invalid transform " + field));
    }
    for (int i = 0; i < lines.size(); i++) {
      if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw (std::runtime_error("bad line for transform " + field));
      }
      for (int j = 0; j < lines[i].size(); j++) {
        if (lines[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
          throw (std::runtime_error("bad value for transform " + field));
        } else {
          T(i, j) = static_cast<double>(lines[i][j]);
        }
      }
    }
    return (T);
  }

  static CameraExtrinsics get_transform(const ros::NodeHandle &nh, const std::string &field,
                                        const CameraExtrinsics &def) {
    CameraExtrinsics T(def);
    try {
      T = get_kalibr_style_transform(nh, field);
    } catch (std::runtime_error &e) {
    }
    return (T);
  }

  static bool parse_camera_pose(CameraPtr cam, const ros::NodeHandle &nh) {
    PoseEstimate pe;
    double posd[3], rvecd[3];
    if (!nh.getParam(cam->name + "/position/x", posd[0]) ||
        !nh.getParam(cam->name + "/position/y", posd[1]) ||
        !nh.getParam(cam->name + "/position/z", posd[2])) {
      return (false); // no position given, fine.
    }
    if (!nh.getParam(cam->name + "/rotvec/x", rvecd[0]) ||
        !nh.getParam(cam->name + "/rotvec/y", rvecd[1]) ||
        !nh.getParam(cam->name + "/rotvec/z", rvecd[2])) {
      bombout("rotvec", cam->name);
    }
    std::vector<double> Rd;
    if (!nh.getParam(cam->name + "/R", Rd)) {
      bombout("R", cam->name);
    }
    if (Rd.size() != 36) {
      bombout("R size != 36", cam->name);
    }
    //const auto R = Eigen::Map<Eigen::Matrix<double, 6, 6> >(Rd)
    const Eigen::Matrix<double, 6, 6> R = Eigen::Map<Eigen::Matrix<double, 6, 6> >(&Rd[0]);
    const Eigen::Vector3d rvec = Eigen::Map<Eigen::Vector3d>(rvecd);
    const Eigen::Vector3d pos = Eigen::Map<Eigen::Vector3d>(posd);
    gtsam::Rot3 rmat(utils::rotmat(rvec));
    gtsam::Point3 t(pos);
    gtsam::Pose3 pose(rmat, t);
    const auto noise = gtsam::noiseModel::Gaussian::SqrtInformation(R, true /*smart*/);
    cam->poseEstimate = PoseEstimate(pose, 0, 0, noise);
    cam->worldPoseKnown = true;
    return (true);
  }

  CameraVec
  Camera::parse_cameras(const ros::NodeHandle &nh) {
    CameraVec cdv;
    std::vector<std::string> camNames = {"cam0", "cam1", "cam2", "cam3",
                                         "cam4", "cam5", "cam6", "cam7"};
    for (const auto cam_idx: irange(0ul, camNames.size())) {
      const auto &cam = camNames[cam_idx];
      XmlRpc::XmlRpcValue lines;
      if (!nh.getParam(cam, lines)) {
        continue;
      }
      CameraPtr camera(new Camera());
      camera->name = cam;
      camera->index = cam_idx;
      CameraIntrinsics &ci = camera->intrinsics;
      if (!nh.getParam(cam + "/camera_model",
                       ci.camera_model)) { bombout("camera_model", cam); }
      if (!nh.getParam(cam + "/distortion_model",
                       ci.distortion_model)) { bombout("distortion_model", cam); }
      if (!nh.getParam(cam + "/distortion_coeffs",
                       ci.distortion_coeffs)) { bombout("distortion_coeffs", cam); }
      if (!nh.getParam(cam + "/intrinsics",  ci.intrinsics)) { bombout("intrinsics", cam); }
      if (!nh.getParam(cam + "/resolution",  ci.resolution)) { bombout("resolution", cam); }
      if (!nh.getParam(cam + "/rostopic",  camera->rostopic)) { bombout("rostopic", cam); }
      if (!nh.getParam(cam + "/tagtopic",  camera->tagtopic)) { bombout("tagtopic", cam); }
      nh.getParam(cam + "/is_static", camera->isStatic);
      if (parse_camera_pose(camera, nh))  {
        std::cout << "world pose known for camera " << cam << std::endl;
      }
      // TODO: don't use CameraExtrinsics, rather use gtsam::Pose3
      camera->T_cam_body = get_transform(nh, cam + "/T_cam_body", CameraExtrinsics::Zero());
      camera->T_cn_cnm1  = get_transform(nh, cam + "/T_cn_cnm1", CameraExtrinsics::Identity());
      const auto &K = ci.intrinsics;
      const auto &D = ci.distortion_coeffs;
      ci.K = (cv::Mat_<double>(3,3) <<
              K[0], 0.0,  K[2],
              0.0,  K[1], K[3],
              0.0,  0.0,  1.0);
      ci.D = cv::Mat_<double>(1, D.size());
      for (int i = 0; i < D.size(); i++) {
        ci.D.at<double>(i) = D[i];
      }
      cdv.push_back(camera);
      double dc[4] = {0, 0, 0, 0};
      for (const auto i: irange(0ul, D.size())) {
        dc[i] = D[i];
      }
      // TODO: only radtan is supported for now
      camera->gtsamCameraModel.reset(
        new gtsam::Cal3DS2(K[0], K[1], 0.0,
                           K[2], K[3],
                           dc[0], dc[1], dc[2], dc[3]));

    }
    return (cdv);
  }

}
