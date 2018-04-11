/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_TAG_GRAPH_H
#define TAGSLAM_TAG_GRAPH_H

#include "tagslam/distance_measurement.h"
#include "tagslam/utils.h"
#include "tagslam/tag.h"
#include "tagslam/camera.h"
#include "tagslam/rigid_body.h"
#include "tagslam/pose_estimate.h"
#include "tagslam/pose_noise.h"
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <opencv2/core/core.hpp>
#include <map>
#include <vector>
#include <memory>
#include <string>

namespace tagslam {
  class TagGraph {
    typedef gtsam::noiseModel::Isotropic::shared_ptr	IsotropicNoisePtr;
  public:
    TagGraph() {};
    virtual ~TagGraph() {};
    TagGraph(const TagGraph&) = delete;
    TagGraph& operator=(const TagGraph&) = delete;

    double getError() { return (optimizerError_); }
    int    getIterations() { return (optimizerIterations_); }
    
    void addTags(const RigidBodyPtr &rb, const TagVec &tags);
    void addDistanceMeasurements(const DistanceMeasurementVec &dmv);
    void observedTags(const CameraPtr &cam, const RigidBodyPtr &rb,
                      const TagVec &tags,
                      unsigned int frame_num);
    void optimize();

    PoseEstimate getCameraPose(const CameraPtr &cam,
                               unsigned int frame_num) const;
    bool getTagRelPose(const RigidBodyPtr &rb, int tagId,
                       gtsam::Pose3 *pose) const;

    PoseEstimate getTagWorldPose(int tagId) const;
    void getTagWorldPoses(std::vector<std::pair<int, gtsam::Pose3>> *poses) const;
    bool getBodyPose(const RigidBodyConstPtr &rb, gtsam::Pose3 *pose,
                     unsigned int frame) const;
    void printDistances() const;
  private:
    bool findInitialTagPose(const Tag &tag, gtsam::Pose3 *pose,
                            PoseNoise *noise) const;
    gtsam::Point3 insertTagType(const TagConstPtr &tag, int corner);

    double tryOptimization(gtsam::Values *result,
                           const gtsam::NonlinearFactorGraph &graph,
                           const gtsam::Values &values,
                           const std::string &verbosity, int maxIter);

    gtsam::Values                 values_;
    gtsam::Values                 optimizedValues_;
    gtsam::NonlinearFactorGraph   graph_;
    std::map<std::string, int>    staticObjects_;
    double                        optimizerError_{0};
    int                           numProjectionFactors_{0};
    int                           optimizerIterations_{0};
  };
}

#endif
