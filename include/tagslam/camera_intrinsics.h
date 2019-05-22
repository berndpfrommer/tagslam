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
  class CameraIntrinsics {
    using string = std::string;
  public:
    friend std::ostream &operator<<(std::ostream &os,
                                    const CameraIntrinsics &ci);
    const std::vector<double> &getDVec() const {
      return (distortionCoeffs_); }
    const std::vector<double> &getKVec() const { return (K_); }
    const cv::Mat &getK() const { return (cvK_); }
    const cv::Mat &getD() const { return (cvD_); }
    const DistortionModel &getDistortionModel() const {
      return (distortionModel_); }
    // i/o functions
    void writeYaml(std::ostream &f, const string &pf) const;
    // static functions
    static CameraIntrinsics parse(XmlRpc::XmlRpcValue config);
  private:
    static CameraIntrinsics parse_no_error(XmlRpc::XmlRpcValue config);
    std::vector<double> distortionCoeffs_;
    std::vector<double> K_; // K Matrix
    std::vector<int>    resolution_;
    DistortionModel     distortionModel_;
    string              cameraModel_;
    cv::Mat cvK_; // precomputed for speed
    cv::Mat cvD_; // precomputed for speed
  };
  std::ostream &operator<<(std::ostream &os, const CameraIntrinsics &ci);
}
