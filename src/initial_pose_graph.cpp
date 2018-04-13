/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/initial_pose_graph.h"
#include "tagslam/resectioning_factor.h"
#include <boost/range/irange.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>

namespace tagslam {
  using namespace boost::random;
  using boost::irange;
  typedef boost::random::mt19937 RandEng;
  typedef boost::random::normal_distribution<double> RandDist;
  typedef boost::random::variate_generator<RandEng, RandDist> RandGen;
  
  static gtsam::Pose3 make_random_pose(RandGen *rgr, RandGen *rgt) {
    gtsam::Point3 t((*rgt)(), (*rgt)(), (*rgt)());
    gtsam::Point3 om((*rgr)(), (*rgr)(), (*rgr)());
    return (gtsam::Pose3(gtsam::Rot3::rodriguez(om.x(),om.y(),om.z()), gtsam::Point3(t)));
  }
  static PoseEstimate try_optimization(const gtsam::Pose3 &startPose, 
                                                 gtsam::NonlinearFactorGraph *graph) {
    gtsam::Symbol P = gtsam::Symbol('P', 0); // pose symbol
    gtsam::Values                 values;
    gtsam::Values                 optimizedValues;
    values.clear();
    values.insert(P, startPose);
    gtsam::LevenbergMarquardtParams lmp;
    lmp.setVerbosity("SILENT");
    const int MAX_ITER = 100;
    lmp.setMaxIterations(MAX_ITER);
    lmp.setAbsoluteErrorTol(1e-7);
    lmp.setRelativeErrorTol(0);
    try {
      gtsam::LevenbergMarquardtOptimizer lmo(*graph, values, lmp);
      optimizedValues = lmo.optimize();
      gtsam::Pose3 op = optimizedValues.at<gtsam::Pose3>(P);
      return (PoseEstimate(op, (double)lmo.error() / graph->size(),
                           (int)lmo.iterations()));
    } catch (const std::exception &e) {
      // bombed out because of cheirality etc
    }
    return (PoseEstimate(startPose, 1e10, MAX_ITER));
  }

  PoseEstimate
  InitialPoseGraph::estimateCameraPose(const CameraPtr &camera,
                                       const std::vector<gtsam::Point3> &wp,
                                       const std::vector<gtsam::Point2> &ip,
                                       const PoseEstimate &initialPose) const {
  	RandEng	randomEngine;
    RandDist distTrans(0, 10.0); // mu, sigma for translation
    RandDist distRot(0, M_PI);	 // mu, sigma for rotations
    RandGen	 rgt(randomEngine, distTrans);	 // random translation generator
    RandGen  rgr(randomEngine, distRot);	   // random angle generator
  
    gtsam::NonlinearFactorGraph   graph;
    PoseEstimate pe;
    if (wp.empty()) {
      return (pe);
    }
    auto pixelNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
    boost::shared_ptr<gtsam::Cal3DS2> cam = camera->gtsamCameraModel;
    graph = gtsam::NonlinearFactorGraph();
    gtsam::Symbol P = gtsam::Symbol('P', 0); // pose symbol
    for (const auto i: boost::irange(0ul, wp.size())) {
      graph.push_back(boost::make_shared<ResectioningFactor>(
                         pixelNoise, P, cam, ip[i], wp[i]));
    }
    gtsam::Pose3 startPose = initialPose.getPose();
    for (const auto i: irange(0, 50)) {
      pe = try_optimization(startPose, &graph);
      if (pe.getError() < 10.0) {
        break;
      }
      startPose = make_random_pose(&rgr, &rgt);
    }
    return (pe);
  }


}  // namespace
