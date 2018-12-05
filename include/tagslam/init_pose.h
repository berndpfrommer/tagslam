/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_INIT_POSE_H
#define TAGSLAM_INIT_POSE_H


#include "tagslam/pose_estimate.h"

#include <opencv2/core/core.hpp>
#include <vector>

namespace tagslam {
  namespace init_pose {
    PoseEstimate pnp(const std::vector<cv::Point3d> &world_points,
                     const std::vector<cv::Point2d> &image_points,
                     const std::vector<cv::Mat>     &T_w_o,
                     const cv::Mat &K,
                     const std::string &distModel,
                     const cv::Mat &D);
  }
}

#endif
