/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/gtsam_optimizer.h"
#include "tagslam/gtsam_utils.h"
#include "tagslam/vertex.h"
#include <boost/graph/filtered_graph.hpp>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace tagslam {

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
#if 0
  void GTSAMOptimizer::add(BoostGraph *graph) {
    BoostGraph &g = *graph;
    ValuesPredicate<BoostGraph> valueFilter(g, true);
    ValuesPredicate<BoostGraph> factorFilter(g, false);
    typedef boost::filtered_graph<
      BoostGraph, boost::keep_all, ValuesPredicate<BoostGraph>> FilteredGraph;
    FilteredGraph values(g, boost::keep_all(), valueFilter);
    FilteredGraph factors(g, boost::keep_all(), factorFilter);

    typedef FilteredGraph::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vp;
    // first add values (must come first!)
    for (vp = vertices(values); vp.first != vp.second; ++vp.first) {
      g[*vp.first].vertex->addToOptimizer(this, *vp.first, &g);
    }
    // then add factors
    for (vp = vertices(factors); vp.first != vp.second; ++vp.first) {
      g[*vp.first].vertex->addToOptimizer(this, *vp.first, &g);
    }
  }
#endif
  
  ValueKey GTSAMOptimizer::addPose(const Transform &p) {
    ValueKey key = generateKey();
    ROS_INFO_STREAM("optimizer: adding pose with key " << key);
    newValues_.insert(key, gtsam_utils::to_gtsam(p));
    return (key);
  }

  static gtsam::noiseModel::Gaussian::shared_ptr  make_pose_noise(double angle, double position) {
    gtsam::Vector sn(6);
    sn << angle,angle,angle,position,position,position;
    return (gtsam::noiseModel::Diagonal::Sigmas(sn));
  }

  FactorKey
  GTSAMOptimizer::addRelativePosePrior(ValueKey key1, ValueKey key2,
                                       const PoseWithNoise &deltaPose) {
    ROS_INFO_STREAM("optimizer: adding relposeprior: " << key1 << " - " << key2);
    newGraph_.push_back(
      gtsam::BetweenFactor<gtsam::Pose3>(key1, key2,
                                         gtsam_utils::to_gtsam(deltaPose.getPose()),
                                         gtsam_utils::to_gtsam(deltaPose.getNoise())));
    return (fullGraph_.size() + newGraph_.size() - 1);
  }

  FactorKey GTSAMOptimizer::addAbsolutePosePrior(ValueKey key,
                                                 const PoseWithNoise &pwn) {
    ROS_INFO_STREAM("optimizer: adding absposeprior: " << key);
    newGraph_.push_back(gtsam::PriorFactor<gtsam::Pose3>
                        (key, gtsam_utils::to_gtsam(pwn.getPose()),
                         gtsam_utils::to_gtsam(pwn.getNoise())));
    return (fullGraph_.size() + newGraph_.size() - 1);
  }

  void GTSAMOptimizer::optimize() {
    ROS_INFO_STREAM("adding values: " << newValues_.size() << " factors: " << newGraph_.size());
    if (newGraph_.size() > 0) {
      fullGraph_ += newGraph_;
      gtsam::ISAM2Result res = isam2_->update(newGraph_, newValues_);
      double err = *res.errorAfter;
      ROS_INFO_STREAM("optimizer error: " << err);
      values_ = isam2_->calculateEstimate();
      //values_.print();
      //fullGraph_.print();
      newGraph_.erase(newGraph_.begin(), newGraph_.end());
      newValues_.clear();
    } else {
      ROS_INFO_STREAM("optimizer: delta graph is 0!");
    }
  }

  void GTSAMOptimizer::optimizeFullGraph() {
    std::cout << "adding new graph of size: " << newGraph_.size() << std::endl;
    fullGraph_ += newGraph_;
    std::cout << "adding new values of size: " << newValues_.size() << std::endl;
    values_.insert(newValues_);
    gtsam::LevenbergMarquardtParams lmp;
    lmp.setVerbosity("SILENT");
    lmp.setMaxIterations(100);
    lmp.setAbsoluteErrorTol(1e-7);
    lmp.setRelativeErrorTol(0);
    gtsam::LevenbergMarquardtOptimizer lmo(fullGraph_, values_, lmp);
    values_ = lmo.optimize();
    ROS_INFO_STREAM("optimizer error: " << lmo.error());
    //fullGraph_.print();
    //values_.print();
    newGraph_.erase(newGraph_.begin(), newGraph_.end());
    newValues_.clear();
  }
  Transform GTSAMOptimizer::getPose(ValueKey key) {
    return (gtsam_utils::from_gtsam(values_.at<gtsam::Pose3>(key)));
  }
  
}  // end of namespace


