/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/logging.h"
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

  static std::shared_ptr<gtsam::ISAM2> make_isam2(OptimizerMode mode) {
    gtsam::ISAM2Params p;
    p.setEnableDetailedResults(true);
    switch (mode) {
    case SLOW:
      // evaluation of nonlinear error is time consuming!
      p.setEvaluateNonlinearError(true);
      // gtsam documentation says "use with caution"
      p.setEnablePartialRelinearizationCheck(false);
      // lower from default of 0.1
      p.relinearizeThreshold = 0.01;
      // don't skip relinearization step
      p.relinearizeSkip = 1;
      break;
    case FAST:
      // when we switch this off, the optimizer error
      // returned is bogus, but it's much faster
      p.setEvaluateNonlinearError(false);
      // never mind the gtsam documentation "use with caution"
      p.setEnablePartialRelinearizationCheck(true);
      // these are the default parameters
      p.relinearizeThreshold = 0.1;
      p.relinearizeSkip = 10;
      break;
    default:
      throw std::runtime_error("invalid optimizer mode!");
      break;
    }
    std::shared_ptr<gtsam::ISAM2> isam2(new gtsam::ISAM2(p));
    return (isam2);
  }

  GTSAMOptimizer::GTSAMOptimizer() {
    verbosity_ = "SILENT";
    isam2_ = make_isam2(mode_);
  }

  GTSAMOptimizer::~GTSAMOptimizer() {
  }

  Optimizer* GTSAMOptimizer::clone() const {
    GTSAMOptimizer *o = new GTSAMOptimizer(*this);
    o->isam2_.reset(new gtsam::ISAM2(*isam2_));
    return (o);
  }

  void GTSAMOptimizer::setMode(OptimizerMode mode) {
    mode_  = mode;
    isam2_ = make_isam2(mode_);
  }

  std::shared_ptr<Cal3DS3> GTSAMOptimizer::getRadTanModel(
    const string &cname, const CameraIntrinsics &ci) {
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
          cname, std::shared_ptr<Cal3DS3>(
            new Cal3DS3(K[0], K[1], K[2], K[3], p1, p2, dc)))).first;
    }
    return (it->second);
  }
  
  std::shared_ptr<Cal3FS2> GTSAMOptimizer::getEquiModel(
    const string &cname, const CameraIntrinsics &ci) {
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
          cname, std::shared_ptr<Cal3FS2>(
            new Cal3FS2(K[0], K[1], K[2], K[3],
                        dc[0], dc[1], dc[2], dc[3])))).first;
    }
    return (it->second);
  }

  static double distance(const gtsam::Point3 &p1, const gtsam::Point3 &p2,
                         gtsam::OptionalJacobian<1, 3> H1 = boost::none,
                         gtsam::OptionalJacobian<1, 3> H2 = boost::none) {
    const gtsam::Point3 d = p1-p2;
    double r = sqrt(d.x() * d.x() + d.y() * d.y() + d.z() * d.z());
    if (H1) *H1 << d.x() / r, d.y() / r, d.z() / r; // jacobian p1
    if (H2) *H2 << -d.x() / r, -d.y() / r, -d.z() / r; // jacobian p2
    return r;
  }

  static double proj(const gtsam::Point3 &p, const gtsam::Point3 &n,
                     gtsam::OptionalJacobian<1, 3> Hp = boost::none,
                     gtsam::OptionalJacobian<1,3> Hn = boost::none) {
    double r = p.x() * n.x() + p.y() * n.y() + p.z() * n.z();
    if (Hp) *Hp << n.x(), n.y(), n.z(); // jacobian w.r.t to p
    if (Hn) *Hn << p.x(), p.y(), p.z(); // jacobian w.r.t to n
    return r;
  }

  ValueKey GTSAMOptimizer::addPose(const Transform &p) {
    ValueKey key = generateKey();
    //ROS_DEBUG_STREAM("optimizer: adding pose with key " << key);
    newValues_.insert(key, gtsam_utils::to_gtsam(p));
    return (key);
  }

  FactorKey
  GTSAMOptimizer::addRelativePosePrior(ValueKey key1, ValueKey key2,
                                       const PoseWithNoise &deltaPose) {
    // key1 = key2 * deltaPose
    newGraph_.push_back(
      gtsam::BetweenFactor<gtsam::Pose3>(
        key1, key2, gtsam_utils::to_gtsam(deltaPose.getPose()),
        gtsam_utils::to_gtsam(deltaPose.getNoise())));
    return (fullGraph_.size() + newGraph_.size() - 1);
  }

  FactorKey GTSAMOptimizer::addAbsolutePosePrior(ValueKey key,
                                                 const PoseWithNoise &pwn) {
    newGraph_.push_back(gtsam::PriorFactor<gtsam::Pose3>
                        (key, gtsam_utils::to_gtsam(pwn.getPose()),
                         gtsam_utils::to_gtsam(pwn.getNoise())));
    return (fullGraph_.size() + newGraph_.size() - 1);
  }

  FactorKey GTSAMOptimizer::addDistanceMeasurement(
    const double d, const double noise,
    Eigen::Vector3d corner1, ValueKey T_w_b1, ValueKey T_b1_o,
    Eigen::Vector3d corner2, ValueKey T_w_b2, ValueKey T_b2_o)
  {
    gtsam::Expression<gtsam::Pose3>  T_w_b_1(T_w_b1);
    gtsam::Expression<gtsam::Pose3>  T_b_o_1(T_b1_o);
    gtsam::Expression<gtsam::Pose3>  T_w_b_2(T_w_b2);
    gtsam::Expression<gtsam::Pose3>  T_b_o_2(T_b2_o);
    gtsam::Expression<gtsam::Point3> X_o_1(corner1);
    gtsam::Expression<gtsam::Point3> X_w_1 =
      gtsam::transform_from(T_w_b_1, gtsam::transform_from(T_b_o_1, X_o_1));
    
    gtsam::Expression<gtsam::Point3> X_o_2(corner2);
    gtsam::Expression<gtsam::Point3> X_w_2 =
      gtsam::transform_from(T_w_b_2, gtsam::transform_from(T_b_o_2, X_o_2));
    gtsam::Expression<double> dist =
      gtsam::Expression<double>(&distance, X_w_1, X_w_2);
    newGraph_.addExpressionFactor(
      dist, d, gtsam::noiseModel::Isotropic::Sigma(1, noise));
    return (fullGraph_.size() + newGraph_.size() - 1);
  }

  FactorKey GTSAMOptimizer::addCoordinateMeasurement(
    const double len, const double noise,
    const Eigen::Vector3d direction,
    const Eigen::Vector3d corner, ValueKey T_w_b_key, ValueKey T_b_o_key)
  {
    gtsam::Expression<gtsam::Pose3>  T_w_b(T_w_b_key);
    gtsam::Expression<gtsam::Pose3>  T_b_o(T_b_o_key);
    gtsam::Expression<gtsam::Point3> X_o(corner);
    gtsam::Expression<gtsam::Point3> X_w =
      gtsam::transform_from(T_w_b, gtsam::transform_from(T_b_o, X_o));
    gtsam::Expression<gtsam::Point3> n(direction);
    gtsam::Expression<double> coord = gtsam::Expression<double>(&proj, X_w, n);
    
    newGraph_.addExpressionFactor(
      coord, len, gtsam::noiseModel::Isotropic::Sigma(1, noise));
    return (fullGraph_.size() + newGraph_.size() - 1);
  }

  std::vector<FactorKey>
  GTSAMOptimizer::addTagProjectionFactor(
    const Eigen::Matrix<double, 4, 2> &imgCorners,
    const Eigen::Matrix<double, 4, 3> &objCorners,
    const string &camName,
    const CameraIntrinsics &ci,
    double pixelNoise,
    ValueKey T_r_c, ValueKey T_w_r, ValueKey T_w_b, ValueKey T_b_o) {
    //ROS_DEBUG_STREAM("gtsam: adding tag proj fac: " << T_r_c << " " <<
    //T_w_r << " " << T_w_b << " " << T_b_o);
    std::vector<FactorKey> keys;
    keys.reserve(4);
    gtsam::Expression<gtsam::Pose3>  T_b_o_fac(T_b_o);
    gtsam::Expression<gtsam::Pose3>  T_w_b_fac(T_w_b);
    gtsam::Expression<gtsam::Pose3>  T_r_c_fac(T_r_c);
    gtsam::Expression<gtsam::Pose3>  T_w_r_fac(T_w_r);

    auto pnit = pixelNoiseMap_.find(pixelNoise);
    if (pnit == pixelNoiseMap_.end()) {
      pnit = pixelNoiseMap_.insert(
        PixelNoiseMap::value_type(
          pixelNoise,gtsam::noiseModel::Isotropic::Sigma(2,pixelNoise))).first;
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
        gtsam::Expression<gtsam::Point2> predict(cK, &Cal3DS3::uncalibrate,xp);
        newGraph_.addExpressionFactor(predict, imgPoint, pnit->second);
        break; }
      case EQUIDISTANT: {
        auto distModel = getEquiModel(camName, ci);
        gtsam::Expression<Cal3FS2> cK(*distModel);
        gtsam::Expression<gtsam::Point2> predict(cK, &Cal3FS2::uncalibrate,xp);
        newGraph_.addExpressionFactor(predict, imgPoint, pnit->second);
        break;  }
      default:
        BOMB_OUT("invalid dist model: " << ci.getDistortionModel());
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
        i->print();  std::cout <<  std::endl << "   values for it: "
                               << std::endl;
        for (const auto &k: i->keys()) {
          values_.at(k).print();
          std::cout << std::endl;
        }

      }
        
    }
    return (me);
  }

  double GTSAMOptimizer::optimize(double deltaError) {
    try {
      return (doOptimize(deltaError));
    } catch (const gtsam::IndeterminantLinearSystemException &e) {
      throw OptimizerException(e.what());
    } catch (const gtsam::CheiralityException &e) {
      throw OptimizerException(e.what());
    }
  }

  double GTSAMOptimizer::doOptimize(double deltaError) {
    ROS_DEBUG_STREAM("incremental optimize new values: " << newValues_.size()
                     << " factors: " << newGraph_.size()
                     << " delta: " << deltaError);
    const bool hasValidError = isam2_->params().isEvaluateNonlinearError();
    if (newGraph_.size() > 0) {
      fullGraph_ += newGraph_;
      gtsam::ISAM2Result res = isam2_->update(newGraph_, newValues_);
      double prevErr = hasValidError ? *res.errorAfter : -1.0;
      for (int i = 0; i < maxIter_ - 1; i++) {
        res = isam2_->update();
        if (!hasValidError) {
          // without nonlinear error calc GTSAM returns garbage
          // so just quit after one iteration
          break;
        }
        // if either there is small improvement
        if (*res.errorAfter < lastError_ + deltaError
            || fabs(*res.errorAfter - prevErr) < 0.01) {
          ROS_DEBUG_STREAM("stopped after iter  " << i << ", changed: "
                           << lastError_  << " -> " << *res.errorAfter
                           << " = " << *res.errorAfter - lastError_
                           << " last iter: " <<  *res.errorAfter - prevErr);
          break;
        } else {
          ROS_DEBUG_STREAM("it " << i << " new err: " << *res.errorAfter
                           << " vs last: " << lastError_ << " +delta: "
                           << lastError_ + deltaError);
        }
        prevErr = *res.errorAfter;
      }
      lastError_ = hasValidError ? *res.errorAfter : -1.0;
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
    return (hasValidError ? lastError_ : -1.0);
  }


  void GTSAMOptimizer::setPose(ValueKey k, const Transform &pose) {
    values_.at<gtsam::Pose3>(k) = gtsam_utils::to_gtsam(pose);
  }

#ifdef DEBUG
  static void print_large_errors(const string &label,
                                 const gtsam::ExpressionFactorGraph &g,
                                 const gtsam::Values &v, double thresh) {
    for (const auto i: g) {
      if (i->error(v) > thresh) {
        std::cout << label << " BIG ERROR: " << i->error(v) << std::endl;
        std::cout << label << " factor: " << std::endl;
        i->print();
        std::cout <<  std::endl << "   values for it: " << std::endl;
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
    try {
      return (doOptimizeFull(force));
    } catch (const gtsam::IndeterminantLinearSystemException &e) {
      throw OptimizerException(e.what());
    } catch (const gtsam::CheiralityException &e) {
      throw OptimizerException(e.what());
    }
  }

  double GTSAMOptimizer::doOptimizeFull(bool force) {
    ROS_DEBUG_STREAM("optimizing full(" << force << ") new fac: " <<
                     newGraph_.size() << ", new val: " << newValues_.size());
    if (newGraph_.empty() && newValues_.empty() && !force) {
      ROS_INFO_STREAM("graph not updated, no need to optimize!");
      return (lastError_);
    }
    fullGraph_ += newGraph_;
    values_.insert(newValues_);

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
    
  PoseNoise GTSAMOptimizer::getMarginal(const ValueKey k)  {
    try {
      auto it = covariances_.find(k);
      if (it == covariances_.end()) {
        gtsam::Marginals marginals(fullGraph_, values_);
        it = covariances_.insert(
          std::map<OptimizerKey,
          gtsam::Matrix>::value_type(k, marginals.marginalCovariance(k))).first;
      }
      const PoseNoise::Matrix6d mat = it->second;
      return (PoseNoise(mat));
    } catch (const gtsam::ValuesKeyDoesNotExist &e) {
      throw (OptimizerException(e.what()));
    }
  }

  void GTSAMOptimizer::transferFullOptimization() {
    isam2_ = make_isam2(mode_);
    isam2_->update(fullGraph_, values_);
  }

  KeyToErrorMap
  GTSAMOptimizer::getErrors(const std::vector<FactorKey> &keys) const {
    KeyToErrorMap ke;
    const gtsam::ExpressionFactorGraph *g;
    const gtsam::Values *values;
    gtsam::ExpressionFactorGraph  testGraph;
    gtsam::Values testValues;
    
    if (newGraph_.empty() && newValues_.empty()) {
      g      = &fullGraph_;
      values = &values_;
    } else {
      // need to first add the new values to the full graph
      testGraph = fullGraph_; // deep copy
      testGraph += newGraph_;
      testValues = values_; // deep copy
      testValues.insert(newValues_);
      g      = &testGraph;
      values = &testValues;
    }
    for (const auto &k: keys) {
      try {
        const auto i = g->at(k);
        double err = i->error(*values);
        ke.insert(KeyToErrorMap::value_type(k, err));
      } catch (const gtsam::ValuesKeyDoesNotExist &e) {
        ROS_WARN_STREAM("cannot get error for key: " << k);
      }
    }
    return (ke);
  }

  void
  GTSAMOptimizer::printFactorError(FactorKey k) const {
    gtsam::ExpressionFactorGraph  testGraph = fullGraph_;
    testGraph += newGraph_;
    gtsam::Values testValues = values_;
    testValues.insert(newValues_);
    const auto i = testGraph.at(k);
    double err = i->error(testValues);
    std::cout << " FACTOR ERROR: " << err << std::endl;
    std::cout << " factor: " << std::endl;
    i->print();  std::cout <<  std::endl
                           << "   corresponding values: " << std::endl;
    for (const auto &fk: i->keys()) {
      testValues.at(fk).print();
      std::cout << std::endl;
    }
  }

  Transform GTSAMOptimizer::getPose(ValueKey key) {
    try {
      const auto it = newValues_.exists<gtsam::Pose3>(key);
      if (it) {
        return (gtsam_utils::from_gtsam(*it));
      }
      return (gtsam_utils::from_gtsam(values_.at<gtsam::Pose3>(key)));
    } catch (const gtsam::ValuesKeyDoesNotExist &e) {
      throw (OptimizerException(e.what()));
    }
  }
  
}  // end of namespace


