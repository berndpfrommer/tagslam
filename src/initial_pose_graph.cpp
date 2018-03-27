/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/initial_pose_graph.h"
#include "tagslam/resectioning_factor.h"
#include <boost/range/irange.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace tagslam {
  
  PoseEstimate
  InitialPoseGraph::estimateCameraPose(const CameraPtr &camera,
                                       const std::vector<gtsam::Point3> &wp,
                                       const std::vector<gtsam::Point2> &ip,
                                       const PoseEstimate &initialPose) const {
    gtsam::Values                 values;
    gtsam::Values                 optimizedValues;
    gtsam::NonlinearFactorGraph   graph;
    PoseEstimate pe;
    if (wp.empty()) {
      return (pe);
    }
    auto pixelNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
    boost::shared_ptr<gtsam::Cal3DS2> cam = camera->gtsamCameraModel;
    gtsam::Symbol P = gtsam::Symbol('P', 0); // pose symbol
    values.clear();
    values.insert(P, initialPose.pose);
    graph = gtsam::NonlinearFactorGraph();
    for (const auto i: boost::irange(0ul, wp.size())) {
      graph.push_back(boost::make_shared<ResectioningFactor>(
                         pixelNoise, P, cam, ip[i], wp[i]));
    }
    gtsam::LevenbergMarquardtParams lmp;
    lmp.setVerbosity("SILENT");
    lmp.setMaxIterations(100);
    lmp.setAbsoluteErrorTol(1e-7);
    lmp.setRelativeErrorTol(0);
    gtsam::LevenbergMarquardtOptimizer lmo(graph, values, lmp);
    optimizedValues = lmo.optimize();
    pe.err = lmo.error() / graph.size();
    pe.numIter = lmo.iterations();
    pe.pose = optimizedValues.at<gtsam::Pose3>(P);
    return (pe);
  }

}  // namespace
