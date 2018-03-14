/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_UTILS_H
#define TAGSLAM_UTILS_H

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <vector>

namespace tagslam {
  namespace utils {
    typedef gtsam::noiseModel::Diagonal::shared_ptr   PoseNoise;
    gtsam::Matrix3 rotmat(const Eigen::Vector3d &rvec);
    PoseNoise      make_pose_noise(const Eigen::Vector3d &a,
                                   const Eigen::Vector3d &p);
    PoseNoise      make_pose_noise(double angle, double position);
    // computes a pose (rotation vector, translation) via homography
    // from world and image points. Distortion is not taken into account,
    // obviously this is just a starting guess.
    gtsam::Pose3 get_init_pose(const std::vector<gtsam::Point3> &world_points,
                               const std::vector<gtsam::Point2> &image_points,
                               const std::vector<double> &intrinsics,
                               const std::string &distModel,
                               const std::vector<double> &distcoeff);
    gtsam::Pose3 get_init_pose_pnp(const std::vector<cv::Point3f> &world_points,
                                   const std::vector<cv::Point2f> &image_points,
                                   const cv::Mat &K, const cv::Mat &D,
                                   bool *success = NULL);
  }
}

#endif
