/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_CAMERA_H
#define TAGSLAM_CAMERA_H

#include "tagslam/camera_intrinsics.h"
#include "tagslam/camera_extrinsics.h"
#include "tagslam/pose_estimate.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <memory>
#include <map>
#include <set>

namespace tagslam {
  struct Camera {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string       name;
    int               index{-1};
    CameraIntrinsics  intrinsics;
    CameraExtrinsics  T_cam_body;
    CameraExtrinsics  T_cn_cnm1;
    std::string       tagtopic;
    bool              isStatic{false};
    PoseEstimate      poseEstimate; // T_w_c
    gtsam::Pose3      optimizedPose;
    int               lastFrameNumber{-1};
    boost::shared_ptr<gtsam::Cal3DS2> gtsamCameraModel;
    typedef std::shared_ptr<Camera> CameraPtr;
    typedef std::shared_ptr<const Camera> CameraConstPtr;
    typedef std::vector<CameraPtr> CameraVec;
    static CameraVec parse_cameras(const ros::NodeHandle &nh);
  };
  using CameraPtr = Camera::CameraPtr;
  using CameraConstPtr = Camera::CameraConstPtr;
  using CameraVec = Camera::CameraVec;
}

#endif
