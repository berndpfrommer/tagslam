/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_CAMERA_H
#define TAGSLAM_CAMERA_H

#include "tagslam/camera_intrinsics.h"
#include "tagslam/camera_extrinsics.h"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <memory>
#include <map>
#include <set>

namespace tagslam {
  class TagSlam;
  struct Camera {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string       name;
    CameraIntrinsics  intrinsics;
    CameraExtrinsics  T_cam_body;
    CameraExtrinsics  T_cn_cnm1;
    std::string       tagtopic;
    static std::vector<Camera, Eigen::aligned_allocator<Camera> > parse_cameras(const ros::NodeHandle &nh);
  };
  typedef std::vector<Camera, Eigen::aligned_allocator<Camera> > CameraVec;
  typedef std::shared_ptr<Camera> CamPtr;
}

#endif
