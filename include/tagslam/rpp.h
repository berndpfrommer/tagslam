/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/geometry.h"
#include <Eigen/Dense>

namespace tagslam {
  namespace rpp {
    typedef Eigen::Matrix<double, 4, 2> ImgPoints;
    typedef Eigen::Matrix<double, 4, 3> ImgPointsH;
    typedef Eigen::Matrix<double, 4, 3> ObjPoints;
    typedef Eigen::Matrix<double, 4, 4> ObjPointsH;
    //
    // computes ratio of error for lowest/(second lowest) minimum error
    // orientation. The lower the ratio, the better is the pose
    // established, the more robust it is to flipping.
    //
    double check_quality(const ImgPoints &ip, const ObjPoints &op,
                         const Transform &T, double *beta_orig,
                         double *beta_min, double *beta_max);
  }
}
