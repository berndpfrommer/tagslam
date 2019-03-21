/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/gtsam_optimizer.h"
#include "tagslam/gtsam_utils.h"
#include "tagslam/vertex.h"
#include "tagslam/cal3ds2u.h"
#include <boost/graph/filtered_graph.hpp>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/expressions.h>
#include <boost/range/irange.hpp>

namespace tagslam {
  using boost::irange;

  static std::shared_ptr<gtsam::ISAM2> make_isam2() {
    gtsam::ISAM2Params p;
    p.setEnableDetailedResults(true);
    p.setEvaluateNonlinearError(true);
    // these two settings were absolutely necessary to
    // make ISAM2 work.
    p.relinearizeThreshold = 0.01;
    p.relinearizeSkip = 1;
    std::shared_ptr<gtsam::ISAM2> isam2(new gtsam::ISAM2(p));
    return (isam2);
  }

  GTSAMOptimizer::GTSAMOptimizer() {
    isam2_ = make_isam2();
  }

  GTSAMOptimizer::~GTSAMOptimizer() {
  }

  template <class G>
  struct ValuesPredicate {
    ValuesPredicate() : graph(NULL) {}
    ValuesPredicate(const G &g, bool v) : graph(&g), isValue(v) {}
    template <typename V>
    bool operator()(const V &v) const {
      return ((*graph)[v].vertex->isValue() == isValue);
    }
    const G *graph;
    bool     isValue;
  };
  
  ValueKey GTSAMOptimizer::addPose(const Transform &p) {
    ValueKey key = generateKey();
    ROS_DEBUG_STREAM("optimizer: adding pose with key " << key);
    newValues_.insert(key, gtsam_utils::to_gtsam(p));
    return (key);
  }
#if 0
  static gtsam::noiseModel::Gaussian::shared_ptr  make_pose_noise(double angle, double position) {
    gtsam::Vector sn(6);
    sn << angle,angle,angle,position,position,position;
    return (gtsam::noiseModel::Diagonal::Sigmas(sn));
  }
#endif  

  FactorKey
  GTSAMOptimizer::addRelativePosePrior(ValueKey key1, ValueKey key2,
                                       const PoseWithNoise &deltaPose) {
    ROS_DEBUG_STREAM("optimizer: adding relposeprior: " << key1 << " - " << key2);
    // key1 = key2 * deltaPose
    newGraph_.push_back(
      gtsam::BetweenFactor<gtsam::Pose3>(
        key1, key2, gtsam_utils::to_gtsam(deltaPose.getPose()),
        gtsam_utils::to_gtsam(deltaPose.getNoise())));
    return (fullGraph_.size() + newGraph_.size());
  }

  FactorKey GTSAMOptimizer::addAbsolutePosePrior(ValueKey key,
                                                 const PoseWithNoise &pwn) {
    ROS_DEBUG_STREAM("optimizer: adding absposeprior: " << key);
    newGraph_.push_back(gtsam::PriorFactor<gtsam::Pose3>
                        (key, gtsam_utils::to_gtsam(pwn.getPose()),
                         gtsam_utils::to_gtsam(pwn.getNoise())));
    return (fullGraph_.size() + newGraph_.size());
  }

  std::shared_ptr<Cal3DS3>
  GTSAMOptimizer::getRadTanModel(const std::string &cname, const CameraIntrinsics2 &ci) {
    // TODO: introduce camera ID and use lookup table!
    auto it = radTanModelMap_.find(cname);
    if (it == radTanModelMap_.end()) {
      const auto &K = ci.getKVec();
      const auto &D = ci.getDVec();
      double dc[6] = {0, 0, 0, 0, 0, 0};
      dc[0]     = D.size() > 0 ? D[0] : 0;
      dc[1]     = D.size() > 1 ? D[1] : 0;
      double p1 = D.size() > 2 ? D[2] : 0;
      double p2 = D.size() > 3 ? D[3] : 0;
      for (const auto i: irange(4ul, D.size())) {
        dc[i - 2] = D[i];
      }
      it = radTanModelMap_.insert(
        RadTanModelMap::value_type(
          cname, std::shared_ptr<Cal3DS3>(new Cal3DS3(K[0], K[1], K[2], K[3], p1, p2, dc)))).first;
    }
    return (it->second);
  }
  
  std::shared_ptr<Cal3FS2>
  GTSAMOptimizer::getEquiModel(const std::string &cname, const CameraIntrinsics2 &ci) {
    // TODO: introduce camera ID and use lookup table!
    auto it = equiModelMap_.find(cname);
    if (it == equiModelMap_.end()) {
      const auto &K = ci.getKVec();
      const auto &D = ci.getDVec();
      double dc[8] = {0, 0, 0, 0, 0, 0, 0, 0};
      for (const auto i: irange(0ul, D.size())) {
        dc[i] = D[i];
      }
      it = equiModelMap_.insert(
        EquiModelMap::value_type(
          cname, std::shared_ptr<Cal3FS2>(new Cal3FS2(K[0], K[1], K[2], K[3], dc[0], dc[1], dc[2], dc[3])))).first;
    }
    return (it->second);
  }

  FactorKey
  GTSAMOptimizer::addTagProjectionFactor(
    const Eigen::Matrix<double, 4, 2> &imgCorners,
    const Eigen::Matrix<double, 4, 3> &objCorners,
    const std::string &camName,
    const CameraIntrinsics2 &ci,
    double pixelNoise,
    ValueKey T_r_c, ValueKey T_w_r, ValueKey T_w_b, ValueKey T_b_o) {
    ROS_DEBUG_STREAM("gtsam: adding tag proj fac: " << T_r_c << " " <<
                    T_w_r << " " << T_w_b << " " << T_b_o);

    gtsam::Expression<gtsam::Pose3>  T_b_o_fac(T_b_o);
    gtsam::Expression<gtsam::Pose3>  T_w_b_fac(T_w_b);
    gtsam::Expression<gtsam::Pose3>  T_r_c_fac(T_r_c);
    gtsam::Expression<gtsam::Pose3>  T_w_r_fac(T_w_r);

    auto pnit = pixelNoiseMap_.find(pixelNoise);
    if (pnit == pixelNoiseMap_.end()) {
      pnit = pixelNoiseMap_.insert(
        PixelNoiseMap::value_type(pixelNoise,
                                  gtsam::noiseModel::Isotropic::Sigma(2, pixelNoise))).first;
    }
    for (const auto i: irange(0, 4)) { // iterate over 4 corners
      const gtsam::Point2 imgPoint(imgCorners(i,0), imgCorners(i,1));
      gtsam::Expression<gtsam::Point3> X_o(objCorners.row(i));
      // transform_from does X_A = T_AB * X_B
      // transform_to   does X_A = T_BA * X_B
      gtsam::Expression<gtsam::Point2> xp =
        gtsam::project(
          gtsam::transform_to(T_r_c_fac,
             gtsam::transform_to(T_w_r_fac,
                gtsam::transform_from(T_w_b_fac,
                   gtsam::transform_from(T_b_o_fac, X_o)))));
      switch (ci.getDistortionModel()) {
      case RADTAN: {
        auto distModel = getRadTanModel(camName, ci);
        gtsam::Expression<Cal3DS3> cK(*distModel);
        gtsam::Expression<gtsam::Point2> predict(cK, &Cal3DS3::uncalibrate, xp);
        newGraph_.addExpressionFactor(predict, imgPoint, pnit->second);
        break; }
      case EQUIDISTANT: {
        auto distModel = getEquiModel(camName, ci);
        gtsam::Expression<Cal3FS2> cK(*distModel);
        gtsam::Expression<gtsam::Point2> predict(cK, &Cal3FS2::uncalibrate, xp);
        newGraph_.addExpressionFactor(predict, imgPoint, pnit->second);
        break;  }
      default:
        ROS_ERROR_STREAM("invalid dist model: " << ci.getDistortionModel());
        throw (std::runtime_error("invalid dist model"));
        break;
      }
    }
    return (fullGraph_.size() + newGraph_.size());
  }

  void GTSAMOptimizer::optimize() {
    ROS_DEBUG_STREAM("optimizer new values: " << newValues_.size()
                    << " factors: " << newGraph_.size());
    //std::cout << "------------ new values: " << std::endl;
    //newValues_.print();
    if (newGraph_.size() > 0) {
      fullGraph_ += newGraph_;
      isam2_->update(newGraph_, newValues_);
      for (int i = 0; i < 20; i++) {
        isam2_->update();
      }
      gtsam::ISAM2Result res = isam2_->update();
      double err = *res.errorAfter;
      ROS_INFO_STREAM("optimizer error: " << err);
      values_ = isam2_->calculateEstimate();
      //std::cout << "---- optimized values: " << std::endl;
      //values_.print();
      //values_.print();
      //fullGraph_.print();
      newGraph_.erase(newGraph_.begin(), newGraph_.end());
      newValues_.clear();
    } else {
      ROS_INFO_STREAM("optimizer: delta graph is 0!");
    }
  }

  void GTSAMOptimizer::optimizeFullGraph() {
    if (newGraph_.empty() && newValues_.empty()) {
      ROS_INFO_STREAM("graph not updated, no need to optimize!");
      return;
    }
    fullGraph_ += newGraph_;
    values_.insert(newValues_);
    gtsam::LevenbergMarquardtParams lmp;
    //lmp.setVerbosity("SILENT");
    lmp.setVerbosity("TERMINATION");
    lmp.setMaxIterations(100);
    lmp.setAbsoluteErrorTol(1e-7);
    lmp.setRelativeErrorTol(0);
    gtsam::LevenbergMarquardtOptimizer lmo(fullGraph_, values_, lmp);
    values_ = lmo.optimize();
    ROS_INFO_STREAM("optimizer error: " << lmo.error());
#ifdef DEBUG_BEFORE_AFTER   
    for (const auto &v : newValues_) {
      std::cout << "----- before: " << std::endl;
      v.value.print();
      std::cout << std::endl;
      std::cout << "----- after: " << std::endl;
      values_.at(v.key).print();
      std::cout << std::endl;
    }
#endif    
    newGraph_.erase(newGraph_.begin(), newGraph_.end());
    newValues_.clear();
  }
  Transform GTSAMOptimizer::getPose(ValueKey key) {
    return (gtsam_utils::from_gtsam(values_.at<gtsam::Pose3>(key)));
  }
  
}  // end of namespace


