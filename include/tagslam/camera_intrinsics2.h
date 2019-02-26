/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#pragma once

#include "tagslam/distortion_model.h"

#include <ros/ros.h>
#include <opencv2/core/core.hpp>

#include <vector>
#include <string>
#include <iostream>

namespace tagslam {
  class CameraIntrinsics2 {
  public:
    friend std::ostream &operator<<(std::ostream &os,
                                    const CameraIntrinsics2 &ci);
    static CameraIntrinsics2 parse(const std::string &prefix,
                                  const ros::NodeHandle &nh);
  private:
    std::vector<double> distortionCoeffs_;
    std::vector<double> K_; // K Matrix
    std::vector<int>    resolution_;
    DistortionModel     distortionModel_;
    std::string         cameraModel_;
    cv::Mat K; // precomputed for speed
    cv::Mat D; // precomputed for speed
  };
  std::ostream &operator<<(std::ostream &os, const CameraIntrinsics2 &ci);
}
