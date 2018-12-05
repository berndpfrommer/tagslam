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
    gtsam::Matrix3 rotmat(const Eigen::Vector3d &rvec);
    // computes a pose (rotation vector, translation) via homography
    // from world and image points.
    bool get_init_pose(const std::vector<cv::Point3d> &world_points,
                       const std::vector<cv::Point2d> &image_points,
                       const cv::Mat &K,
                       const std::string &distModel,
                       const cv::Mat &D,
                       cv::Mat *rvec,
                       cv::Mat *tvec);
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
    //
    // returns the shorter side of the square into which
    // all pixels fall.
    //
    double get_pixel_range(const std::vector<gtsam::Point2> &ip);

    //
    // tests if all world points are in front of the camera
    //
    bool has_negative_z(const gtsam::Pose3 &T_c_w,
                        const std::vector<gtsam::Point3>&wp);
  }
}

#endif
