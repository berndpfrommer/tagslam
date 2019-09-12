/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/geometry.h"
#include "tagslam/distortion_model.h"
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <string>
#include <utility>

namespace tagslam {
  namespace init_pose {
    struct Params {
      double minViewingAngle;
      double ambiguityAngleThreshold;
      double maxAmbiguityRatio;
    };
    std::pair<Transform, bool> pose_from_4(
      const Eigen::Matrix<double, 4,2> & imgPoints,
      const Eigen::Matrix<double, 4,3> & objPoints,
      const cv::Mat &K, DistortionModel distModel,
      const cv::Mat &D, const Params &params);
    std::string cv_type_to_str(int type);
    std::string cv_info(const cv::Mat &m);
    std::string print_as_python_mat(const cv::Mat &m);
  }
}
