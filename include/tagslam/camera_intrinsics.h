/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_CAMERA_INTRINSICS_H
#define TAGSLAM_CAMERA_INTRINSICS_H

#include <vector>
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>

namespace tagslam {
  struct CameraIntrinsics {
    CameraIntrinsics() {
      intrinsics.resize(4);
      distortion_coeffs.resize(4);
      resolution.resize(2);
    }
    std::vector<double> distortion_coeffs;
    std::vector<double> intrinsics; // K Matrix
    std::vector<int>    resolution;
    std::string distortion_model;
    std::string camera_model;
    cv::Mat K; // precomputed for speed
    cv::Mat D; // precomputed for speed
  };
  std::ostream &operator<<(std::ostream &os, const CameraIntrinsics &ci);
}
#endif
