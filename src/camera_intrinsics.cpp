/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/camera_intrinsics.h"
#include "tagslam/xml.h"
#include "tagslam/logging.h"

#include <ros/console.h>
#include <string>
#include <sstream>
#include <vector>
#include <ctime>

using std::string;

namespace tagslam {

  static std::map<string, DistortionModel> distMap = {
    {"rad_tan", RADTAN}, {"radtan", RADTAN}, {"plumb_bob", RADTAN},
    {"equidistant", EQUIDISTANT}, {"equi", EQUIDISTANT},
    {"fisheye", EQUIDISTANT}};

  CameraIntrinsics
  CameraIntrinsics::parse_no_error(XmlRpc::XmlRpcValue config) {
    CameraIntrinsics ci;
    ci.cameraModel_= xml::parse<string>(config, "camera_model");
    const string distModel = xml::parse<string>(config, "distortion_model");
    if (distMap.count(distModel) == 0) {
      BOMB_OUT("unknown distortion model: " << distModel);
    }
    ci.distortionModel_ = distMap[distModel];
    ci.distortionCoeffs_ =
      xml::parse_container<std::vector<double>>(config, "distortion_coeffs");
    ci.K_ = xml::parse_container<std::vector<double>>(config, "intrinsics");
    ci.resolution_ =
      xml::parse_container<std::vector<int>>(config, "resolution");
    // precompute K and D
    ci.cvK_ = (cv::Mat_<double>(3,3) <<
               ci.K_[0], 0.0,  ci.K_[2],
               0.0,  ci.K_[1], ci.K_[3],
               0.0,   0.0,  1.0);
    const auto &D = ci.distortionCoeffs_;
    ci.cvD_ = cv::Mat_<double>(1, D.size());
    for (unsigned int i = 0; i < D.size(); i++) {
      ci.cvD_.at<double>(i) = D[i];
    }
    return (ci);
  }
  CameraIntrinsics
  CameraIntrinsics::parse(XmlRpc::XmlRpcValue config) {
    try {
      return (parse_no_error(config));
    } catch (const XmlRpc::XmlRpcException &e) {
      ROS_ERROR_STREAM("error parsing camera intrinsics: " << e.getMessage());
      throw e;
    }
  }

}
