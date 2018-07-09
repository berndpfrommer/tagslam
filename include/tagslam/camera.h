/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_CAMERA_H
#define TAGSLAM_CAMERA_H

#include "tagslam/camera_intrinsics.h"
#include "tagslam/camera_extrinsics.h"
#include "tagslam/pose_estimate.h"
#include "gtsam_equidistant/Cal3FS2.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <memory>
#include <map>
#include <set>

namespace tagslam {
  struct RigidBody;
  struct Camera {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string       name;
    int               index{-1};
    CameraIntrinsics  intrinsics;
    CameraExtrinsics  T_cam_body;
    CameraExtrinsics  T_cn_cnm1;
    std::string       rostopic;
    std::string       tagtopic;
    std::string       frame_id;
    bool              hasPosePrior{false};
    PoseEstimate      poseEstimate; // T_r_c
    gtsam::Pose3      optimizedPose;
    int               lastFrameNumber{-1};
    std::string       rig_body; // string with name
    std::shared_ptr<RigidBody>        rig;
    boost::shared_ptr<Cal3FS2>        equidistantModel;
    boost::shared_ptr<gtsam::Cal3DS2> radtanModel;
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
