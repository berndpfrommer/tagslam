/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include "tagslam/optimizer.h"
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include "gtsam_distortion/Cal3FS2.h"
#include "gtsam_distortion/Cal3DS3.h"
#include <unordered_map>

namespace tagslam {
  class GTSAMOptimizer: public Optimizer {
  public:
    using string = std::string;
    GTSAMOptimizer();
    ~GTSAMOptimizer();
    // ---- implement Optimizer interface
    double    optimize(double deltaError) override;
    double    optimizeFull(bool force = false) override;
    void      transferFullOptimization() override;
    double    errorFull() override;
    KeyToErrorMap getErrors(const std::vector<FactorKey> &keys) const override;
    void      printFactorError(FactorKey k) const override;

    double    getMaxError() const override;
    void      setErrorThreshold(double th) override { errorThreshold_ = th; }
    void      setVerbosity(const string &v) override { verbosity_ = v;}
    void      setMode(OptimizerMode mode) override;

    Optimizer *clone() const override;
    Transform getPose(ValueKey key) override;
    PoseNoise getMarginal(const ValueKey k) override;
    ValueKey  addPose(const Transform &pose) override;
    FactorKey addRelativePosePrior(ValueKey key1, ValueKey key2,
                                   const PoseWithNoise &deltaPose) override;
    FactorKey addAbsolutePosePrior(ValueKey key,
                                   const PoseWithNoise &pose) override;
    std::vector<FactorKey> addTagProjectionFactor(
      const Eigen::Matrix<double, 4, 2> &imgCorners,
      const Eigen::Matrix<double, 4, 3> &objCorners,
      const string &cameraName,
      const CameraIntrinsics &ci,
      double pixelNoise,
      ValueKey T_c_r, ValueKey T_r_w, ValueKey T_w_b, ValueKey T_b_o) override;
    FactorKey addDistanceMeasurement(
      const double distance, const double noise,
      Eigen::Vector3d corner1, ValueKey T_w_b1, ValueKey T_b1_o,
      Eigen::Vector3d corner2, ValueKey T_w_b2, ValueKey T_b2_o) override;
    FactorKey addCoordinateMeasurement(
      const double len, const double noise, const Eigen::Vector3d direction,
      const Eigen::Vector3d corner,
      ValueKey T_w_b_key, ValueKey T_b_o_key) override;

    gtsam::ExpressionFactorGraph  &getGraph() { return (newGraph_); }
    void setPose(ValueKey k, const Transform &pose) override;

  private:
    inline ValueKey generateKey() { return (++key_); } // starts at 1!
    typedef std::unordered_map<string,std::shared_ptr<Cal3FS2>> EquiModelMap;
    typedef std::unordered_map<string,std::shared_ptr<Cal3DS3>> RadTanModelMap;
    typedef std::unordered_map<
      double, gtsam::noiseModel::Isotropic::shared_ptr> PixelNoiseMap;
    std::shared_ptr<Cal3FS2> getEquiModel(
      const string &cname, const CameraIntrinsics &ci);
    std::shared_ptr<Cal3DS3> getRadTanModel(
      const string &cname, const CameraIntrinsics &ci);
    double checkForLargeErrors(double thresh) const;
    double doOptimize(double deltaError);
    double doOptimizeFull(bool force);
    // ------------ variables ------------
    ValueKey                      key_{0};
    string                        verbosity_;
    gtsam::Values                 values_;
    gtsam::Values                 newValues_;
    std::shared_ptr<gtsam::ISAM2> isam2_;
    gtsam::ExpressionFactorGraph  fullGraph_;
    gtsam::ExpressionFactorGraph  newGraph_;
    int                           maxIter_{20};
    double                        lastError_{0};
    double                        errorThreshold_{2.0};
    RadTanModelMap                radTanModelMap_;
    EquiModelMap                  equiModelMap_;
    PixelNoiseMap                 pixelNoiseMap_;
    OptimizerMode                 mode_{SLOW};
    std::map<OptimizerKey, gtsam::Matrix, std::less<OptimizerKey>,
             Eigen::aligned_allocator<std::pair<const OptimizerKey,
                                                gtsam::Matrix>>> covariances_;
  };
}
