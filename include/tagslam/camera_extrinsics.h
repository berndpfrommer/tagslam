/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_CAMERA_EXTRINSICS_H
#define TAGSLAM_CAMERA_EXTRINSICS_H

#include <vector>
#include <Eigen/Core>

namespace tagslam {
  using  CameraExtrinsics = Eigen::Matrix<double, 4, 4>;
  typedef std::vector<CameraExtrinsics, Eigen::aligned_allocator<CameraExtrinsics> > CameraExtrinsicsVec;
}
#endif
