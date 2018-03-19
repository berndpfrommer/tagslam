/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_GRAPH_CAM_H
#define TAGSLAM_GRAPH_CAM_H

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>
#include <memory>
#include <string>
#include <boost/range/irange.hpp>

namespace tagslam {
  struct GraphCam {
    GraphCam(const std::vector<double> &intr = std::vector<double>(),
             const std::string &distModel  = "radtan",
             const std::vector<double> &distCoeff = std::vector<double>()) {
      if (distCoeff.size() > 4) {
        throw std::runtime_error("max of 4 dist coeff is supported!");
      }
      double dc[4] = {0, 0, 0, 0};
      for (const auto i: boost::irange(0ul, distCoeff.size())) {
        dc[i] = distCoeff[i];
      }
      cameraModel.reset(new gtsam::Cal3DS2(intr[0], intr[1], 0.0,
                                           intr[2], intr[3],
                                           dc[0], dc[1], dc[2], dc[3]));
    }
    boost::shared_ptr<gtsam::Cal3DS2> cameraModel;
    gtsam::Pose3        lastPose;
    bool                hasValidPose{false};
  };
}
#endif
