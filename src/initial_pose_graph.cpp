/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/initial_pose_graph.h"
#include "tagslam/resectioning_factor.h"
#include <boost/range/irange.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace tagslam {
  void InitialPoseGraph::addCamera(int cam_idx,
                                   const std::vector<double> &intr,
                                   const std::string &distModel,
                                   const std::vector<double> &distCoeff) {
    if (cam_idx != cameras_.size()) {
      std::cout << "ERROR: camera index must be consecutive!" << std::endl;
      throw std::runtime_error("invalid cam idx: " + std::to_string(cam_idx));
    }
    cameras_.push_back(GraphCam(intr, distModel, distCoeff));
  }

  bool
  InitialPoseGraph::estimateCameraPose(int cam_idx,
                                       const std::vector<gtsam::Point3> &wp,
                                       const std::vector<gtsam::Point2> &ip,
                                       gtsam::Pose3 const &initialPose,
                                       gtsam::Pose3 *pose, double *err,
                                       int *numIter) {
    auto pixelNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
    boost::shared_ptr<gtsam::Cal3DS2> cam = cameras_[cam_idx].cameraModel;
    gtsam::Symbol P = gtsam::Symbol('P', 0); // pose symbol
    values_.clear();
    values_.insert(P, initialPose);
    graph_ = gtsam::NonlinearFactorGraph();
    for (const auto i: boost::irange(0ul, wp.size())) {
      graph_.push_back(boost::make_shared<ResectioningFactor>(
                         pixelNoise, P, cam, ip[i], wp[i]));
    }
    gtsam::LevenbergMarquardtParams lmp;
    //lmp.setVerbosity("TERMINATION");
    lmp.setVerbosity("SILENT");
    lmp.setMaxIterations(100);
    lmp.setAbsoluteErrorTol(1e-7);
    lmp.setRelativeErrorTol(0);
#if 0    
    std::cout << "-------------- initial values: " << std::endl;
    graph_.print();
    values_.print();
#endif    
    gtsam::LevenbergMarquardtOptimizer lmo(graph_, values_, lmp);
    optimizedValues_ = lmo.optimize();
#if 0    
    std::cout << "-------------- optimized values: " << std::endl;
    optimizedValues_.print();
#endif    
    *err = lmo.error() / graph_.size();
    *numIter = lmo.iterations();
    *pose = optimizedValues_.at<gtsam::Pose3>(P);
    return (true);
  }

}  // namespace
