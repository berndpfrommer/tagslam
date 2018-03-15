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
#include <opencv2/calib3d/calib3d.hpp>
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
    // from world and image points.
    gtsam::Pose3 get_init_pose(const std::vector<gtsam::Point3> &world_points,
                               const std::vector<gtsam::Point2> &image_points,
                               const std::vector<double> &intrinsics,
                               const std::string &distModel,
                               const std::vector<double> &distcoeff);
    //
    // Will return T_c_w, i.e. world-to-camera transform
    //
    bool get_init_pose_pnp(const std::vector<cv::Point3d> &world_points,
                           const std::vector<cv::Point2d> &image_points,
                           const cv::Mat &K,
                           const std::string &distModel,
                           const cv::Mat &D,
                           cv::Mat *rvec,
                           cv::Mat *tvec);
    //
    // project points
    //
    void project_points(const std::vector<cv::Point3d> &wp,
                        const cv::Mat &rvec,
                        const cv::Mat &tvec,
                        const cv::Mat &K,
                        const std::string &distModel,
                        const cv::Mat &D, 
                        std::vector<cv::Point2d> *ip);
    //
    // reprojection error
    //
    double reprojection_error(const std::vector<cv::Point3d> &wp,
                              const std::vector<cv::Point2d> &ip,
                              const cv::Mat &rvec,
                              const cv::Mat &tvec,
                              const cv::Mat &K,
                              const std::string &distModel,
                              const cv::Mat &D);

  }
}

#endif
