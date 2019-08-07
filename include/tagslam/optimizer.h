/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include "tagslam/value_key.h"
#include "tagslam/factor_key.h"
#include "tagslam/geometry.h"
#include "tagslam/camera_intrinsics.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/optimizer_mode.h"

#include <map>

namespace tagslam {
  class OptimizerException: public std::runtime_error {
  public:
    OptimizerException(const std::string &what) : std::runtime_error(what) {
    };
  };
  typedef std::map<FactorKey, double> KeyToErrorMap;
  class Optimizer {
  public:
    using string = std::string;
    Optimizer() {}
    virtual ~Optimizer() {};

    virtual double optimize(double deltaError) = 0;
    virtual double optimizeFull(bool force = false) = 0;
    virtual void   transferFullOptimization() = 0;
    virtual double errorFull() = 0;
    virtual KeyToErrorMap getErrors(
      const std::vector<FactorKey> &keys) const = 0;
    virtual void   printFactorError(FactorKey k) const = 0;
    virtual double getMaxError() const = 0;
    virtual void   setErrorThreshold(double th) = 0;
    virtual void   setVerbosity(const string &v) = 0;
    virtual void   setMode(OptimizerMode mode) = 0;
    // makes deep copy
    virtual Optimizer *clone() const = 0;
    // retrieves the optimized pose for a given key
    virtual Transform getPose(ValueKey key) = 0;
    // retrieves marginal for given key
    virtual PoseNoise getMarginal(const ValueKey k) = 0;
    // adds the starting guess for a new value (e.g. camera pose)
    virtual ValueKey addPose(const Transform &pose) = 0;
    // relative pose prior, i.e. err = ||Pose(key1) - deltaPose * Pose(key2)||
    virtual FactorKey addRelativePosePrior(ValueKey key1, ValueKey key2,
                                           const PoseWithNoise &deltaPose) = 0;
    // absolute pose prior, i.e. err = ||Pose(key) - pose||
    virtual FactorKey addAbsolutePosePrior(ValueKey key,
                                           const PoseWithNoise &pose) = 0;
    // tag projection factor:
    //    T_c_o   = T_c_r * T_r_w * T_w_b * T_b_o;
    //    u_proj_i = K * rad_dist(T_c_o * X_i)   where i = 1..4 (corners)
    //    err = sum_i  ||u_proj_i - u||
    virtual std::vector<FactorKey> addTagProjectionFactor(
      // u = image points  (2d)
      const Eigen::Matrix<double, 4, 2> &u,
      // X = object points (3d, but in plane with z = 0)
      const Eigen::Matrix<double, 4, 3> &X,
      const string &cameraName,
      const CameraIntrinsics &ci,
      double pixelNoise,
      ValueKey T_c_r, ValueKey T_r_w, ValueKey T_w_b, ValueKey T_b_o) = 0;
 
    // distance measurement factor:
    // err = || T_w_b2 * T_b2_o * corner2 - T_w_b1 * T_b1_o * corner1||
    virtual FactorKey addDistanceMeasurement(
      const double distance, const double noise,
      Eigen::Vector3d corner1, ValueKey T_w_b1, ValueKey T_b1_o,
      Eigen::Vector3d corner2, ValueKey T_w_b2, ValueKey T_b2_o) = 0;

    // coordinate measurement factor
    // err = || n * (T_w_b * T_b_o * corner) - len||
    virtual FactorKey addCoordinateMeasurement(
      const double len, const double noise,
      const Eigen::Vector3d direction,
      const Eigen::Vector3d corner,
      ValueKey T_w_b_key, ValueKey T_b_o_key) = 0;

    // for debugging, allow direct setting of value
    virtual void setPose(ValueKey k, const Transform &pose) = 0;
  private:
  };
}
