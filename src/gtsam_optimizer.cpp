/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/gtsam_optimizer.h"
#include "tagslam/gtsam_utils.h"
#include "tagslam/vertex.h"
#include "tagslam/cal3ds2u.h"
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/Marginals.h>
#include <boost/range/irange.hpp>

#undef DEBUG

namespace tagslam {
  using boost::irange;

  static std::shared_ptr<gtsam::ISAM2> make_isam2() {
    gtsam::ISAM2Params p;
    p.setEnableDetailedResults(true);
    p.setEvaluateNonlinearError(true);
    // these two settings were absolutely necessary to
    // make ISAM2 work.
    //p.relinearizeThreshold = 0.01;
    // p.relinearizeSkip = 1;
    p.relinearizeThreshold = 0.05;
    p.relinearizeSkip = 10;
    std::shared_ptr<gtsam::ISAM2> isam2(new gtsam::ISAM2(p));
    return (isam2);
  }

  GTSAMOptimizer::GTSAMOptimizer() {
    verbosity_ = "SILENT";
    isam2_ = make_isam2();
  }

  GTSAMOptimizer::~GTSAMOptimizer() {
  }

  ValueKey GTSAMOptimizer::addPose(const Transform &p) {
    ValueKey key = generateKey();
    ROS_DEBUG_STREAM("optimizer: adding pose with key " << key);
    newValues_.insert(key, gtsam_utils::to_gtsam(p));
    return (key);
  }

  FactorKey
  GTSAMOptimizer::addRelativePosePrior(ValueKey key1, ValueKey key2,
                                       const PoseWithNoise &deltaPose) {
    ROS_DEBUG_STREAM("optimizer: adding relposeprior: " << key1 << " - " << key2);
    // key1 = key2 * deltaPose
    newGraph_.push_back(
      gtsam::BetweenFactor<gtsam::Pose3>(
        key1, key2, gtsam_utils::to_gtsam(deltaPose.getPose()),
        gtsam_utils::to_gtsam(deltaPose.getNoise())));
    return (fullGraph_.size() + newGraph_.size() - 1);
  }

  FactorKey GTSAMOptimizer::addAbsolutePosePrior(ValueKey key,
                                                 const PoseWithNoise &pwn) {
    ROS_DEBUG_STREAM("optimizer: adding absposeprior: " << key);
    newGraph_.push_back(gtsam::PriorFactor<gtsam::Pose3>
                        (key, gtsam_utils::to_gtsam(pwn.getPose()),
                         gtsam_utils::to_gtsam(pwn.getNoise())));
    return (fullGraph_.size() + newGraph_.size() - 1);
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

  std::vector<FactorKey>
  GTSAMOptimizer::addTagProjectionFactor(
    const Eigen::Matrix<double, 4, 2> &imgCorners,
    const Eigen::Matrix<double, 4, 3> &objCorners,
    const std::string &camName,
    const CameraIntrinsics2 &ci,
    double pixelNoise,
    ValueKey T_r_c, ValueKey T_w_r, ValueKey T_w_b, ValueKey T_b_o) {
    ROS_DEBUG_STREAM("gtsam: adding tag proj fac: " << T_r_c << " " <<
                    T_w_r << " " << T_w_b << " " << T_b_o);
    std::vector<FactorKey> keys;
    keys.reserve(4);
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
      keys.push_back(fullGraph_.size() + newGraph_.size() - 1);
    }
    return (keys);
  }

  double GTSAMOptimizer::getMaxError() const {
    double me(0);
    for (const auto i: fullGraph_) {
      double e = i->error(values_);
      if (e > me) {
        me = e;
      }
    }
    return (me);
  }
  
  double GTSAMOptimizer::checkForLargeErrors(double thresh) const {
    double me(0);
    for (const auto i: fullGraph_) {
      double e = i->error(values_);
      if (e > me) {
        me = e;
      }
      if (e > thresh) {
        ROS_DEBUG_STREAM("graph has large err: " << i->error(values_));
        std::cout << " factor: " << std::endl;
        i->print();  std::cout <<  std::endl << "   values for it: " << std::endl;
        for (const auto &k: i->keys()) {
          values_.at(k).print();
          std::cout << std::endl;
        }

      }
        
    }
    return (me);
  }

  double GTSAMOptimizer::optimize(double deltaError) {
    ROS_DEBUG_STREAM("incremental optimize new values: " << newValues_.size()
                     << " factors: " << newGraph_.size() << " delta: " << deltaError);
    if (newGraph_.size() > 0) {
      fullGraph_ += newGraph_;
      gtsam::ISAM2Result res = isam2_->update(newGraph_, newValues_);
      double prevErr = *res.errorAfter;
      for (int i = 0; i < maxIter_ - 1; i++) {
        res = isam2_->update();
        // if either there is small improvement
        if (*res.errorAfter < lastError_ + deltaError
            || fabs(*res.errorAfter - prevErr) < 0.01) {
          ROS_DEBUG_STREAM("stopped after iteration " << i << " now: " << *res.errorAfter << " change: " << *res.errorAfter - prevErr << " last: " << lastError_);
          break;
        } else {
          ROS_DEBUG_STREAM("new err: " << *res.errorAfter << " vs last: " << lastError_ << " +delta: " << lastError_ + deltaError);
        }
        prevErr = *res.errorAfter;
      }
      lastError_ = *res.errorAfter;
      values_ = isam2_->calculateEstimate();
//#define DEBUG_BEFORE_AFTER      
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
    } else {
      ROS_INFO_STREAM("optimizer: delta graph is 0!");
    }
    return (lastError_);
  }


  void GTSAMOptimizer::setPose(ValueKey k, const Transform &pose) {
    values_.at<gtsam::Pose3>(k) = gtsam_utils::to_gtsam(pose);
  }

#ifdef DEBUG
  static void print_large_errors(const std::string &label,
                                 const gtsam::ExpressionFactorGraph &g,
                                 const gtsam::Values &v, double thresh) {
    for (const auto i: g) {
      if (i->error(v) > thresh) {
        std::cout << label << " BIG ERROR: " << i->error(v) << std::endl;
        std::cout << label << " factor: " << std::endl;
        i->print();  std::cout <<  std::endl << "   values for it: " << std::endl;
        for (const auto &k: i->keys()) {
          v.at(k).print();
          std::cout << std::endl;
        }
      } else {
        //std::cout << label << " SMALL ERROR: " << i->error(v) << std::endl;
        //std::cout << label << " factor: " << std::endl;
        //i->print();  std::cout <<  std::endl << std::endl;
      }
    }
  }
#endif

  double GTSAMOptimizer::errorFull() {
    gtsam::ExpressionFactorGraph  testGraph = fullGraph_;
    testGraph += newGraph_;
    gtsam::Values testValues = values_;
    testValues.insert(newValues_);
#ifdef DEBUG    
    print_large_errors("errorFull", testGraph, testValues, 10.0);
#endif    
    return (testGraph.error(testValues));
  }

  double GTSAMOptimizer::optimizeFull(bool force) {
    ROS_DEBUG_STREAM("optimizing full(" << force << ") new fac: " <<
                     newGraph_.size() << ", new val: " << newValues_.size());
    if (newGraph_.empty() && newValues_.empty() && !force) {
      ROS_INFO_STREAM("graph not updated, no need to optimize!");
      return (lastError_);
    }
    fullGraph_ += newGraph_;
    values_.insert(newValues_);

    //print_large_errors("before full opt", fullGraph_, values_, 10.0);
    gtsam::LevenbergMarquardtParams lmp;
    lmp.setVerbosity(verbosity_);
    lmp.setMaxIterations(100);
    lmp.setAbsoluteErrorTol(1e-7);
    lmp.setRelativeErrorTol(0);
    gtsam::LevenbergMarquardtOptimizer lmo(fullGraph_, values_, lmp);
    values_ = lmo.optimize();
    lastError_ = lmo.error();
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
    return (lastError_);
  }

  PoseNoise2 GTSAMOptimizer::getMarginal(const ValueKey k)  {
    auto it = covariances_.find(k);
    if (it == covariances_.end()) {
      gtsam::Marginals marginals(fullGraph_, values_);
      it = covariances_.insert(
        std::map<OptimizerKey,
        gtsam::Matrix>::value_type(k, marginals.marginalCovariance(k))).first;
    }
    const Matrix6d mat = it->second;
    return (PoseNoise2(mat));
  }

  void GTSAMOptimizer::transferFullOptimization() {
    isam2_ = make_isam2();
    isam2_->update(fullGraph_, values_);
  }

  double GTSAMOptimizer::getError(FactorKey k) const {
    gtsam::ExpressionFactorGraph  testGraph = fullGraph_;
    testGraph += newGraph_;
    gtsam::Values testValues = values_;
    testValues.insert(newValues_);
    const auto i = testGraph.at(k);
    double err = i->error(testValues);
#ifdef DEBUG
    if (err > 100.0) {
      std::cout << " FACTOR WITH LARGE ERROR: " << err << std::endl;
      std::cout << " factor: " << std::endl;
      i->print();  std::cout <<  std::endl << "   values for it: " << std::endl;
      for (const auto &fk: i->keys()) {
        testValues.at(fk).print();
        std::cout << std::endl;
      }
    }
#endif    
    return (err);
  }

  Transform GTSAMOptimizer::getPose(ValueKey key) {
    const auto it = newValues_.exists<gtsam::Pose3>(key);
    if (it) {
      return (gtsam_utils::from_gtsam(*it));
    }
    return (gtsam_utils::from_gtsam(values_.at<gtsam::Pose3>(key)));
  }
  
}  // end of namespace


