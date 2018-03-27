/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_INITIAL_POSE_GRAPH_H
#define TAGSLAM_INITIAL_POSE_GRAPH_H

#include "tagslam/graph_cam.h"
#include "tagslam/pose_estimate.h"
#include "tagslam/camera.h"
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <vector>
#include <memory>
#include <string>

namespace tagslam {
  class InitialPoseGraph {
    //typedef gtsam::noiseModel::Isotropic::shared_ptr	IsotropicNoisePtr;
  public:
    InitialPoseGraph() {};
    virtual ~InitialPoseGraph() {};
    InitialPoseGraph(const InitialPoseGraph&) = delete;
    InitialPoseGraph& operator=(const InitialPoseGraph&) = delete;

    PoseEstimate
    estimateCameraPose(const CameraPtr &camera,
                       const std::vector<gtsam::Point3> &wp,
                       const std::vector<gtsam::Point2> &ip,
                       const PoseEstimate &initialPose) const;
  private:
  };
}

#endif
