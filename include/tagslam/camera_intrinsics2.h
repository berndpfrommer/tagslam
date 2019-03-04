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
    const std::vector<double> &getDVec() const {
      return (distortionCoeffs_); }
    const std::vector<double> &getKVec() const { return (K_); }
    const cv::Mat &getK() const { return (cvK_); }
    const cv::Mat &getD() const { return (cvD_); }
    const DistortionModel &getDistortionModel() const { return (distortionModel_); }
  private:
    std::vector<double> distortionCoeffs_;
    std::vector<double> K_; // K Matrix
    std::vector<int>    resolution_;
    DistortionModel     distortionModel_;
    std::string         cameraModel_;
    cv::Mat cvK_; // precomputed for speed
    cv::Mat cvD_; // precomputed for speed
  };
  std::ostream &operator<<(std::ostream &os, const CameraIntrinsics2 &ci);
}
