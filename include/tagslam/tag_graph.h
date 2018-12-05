/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_TAG_GRAPH_H
#define TAGSLAM_TAG_GRAPH_H

#include "tagslam/distance_measurement.h"
#include "tagslam/position_measurement.h"
#include "tagslam/utils.h"
#include "tagslam/tag.h"
#include "tagslam/camera.h"
#include "tagslam/rigid_body.h"
#include "tagslam/pose_estimate.h"
#include "tagslam/pose_noise.h"
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <opencv2/core/core.hpp>
#include <map>
#include <vector>
#include <memory>
#include <string>

namespace tagslam {
  class TagGraph {
    typedef gtsam::noiseModel::Isotropic::shared_ptr	IsotropicNoisePtr;
  public:
    TagGraph();
    virtual ~TagGraph() {};
    TagGraph(const TagGraph&) = delete;
    TagGraph& operator=(const TagGraph&) = delete;

    double getError() { return (optimizerError_); }
    int    getIterations() { return (optimizerIterations_); }
    void   setPixelNoise(double numPix);

    void addTags(const RigidBodyPtr &rb, const TagVec &tags);
    void addCamera(const CameraConstPtr &cam);
    bool addDistanceMeasurement(const RigidBodyPtr &rb1,
                                const RigidBodyPtr &rb2,
                                const TagConstPtr &tag1,
                                const TagConstPtr &tag2,
                                const DistanceMeasurement &dm);
    bool addPositionMeasurement(const RigidBodyPtr &rb,
                                const TagConstPtr &tag,
                                const PositionMeasurement &m);

    void observedTags(const CameraPtr &cam, const RigidBodyPtr &rb,
                      const TagVec &tags,
                      unsigned int frame_num);
    void optimize();
    void computeMarginals();

    PoseEstimate getCameraPose(const CameraPtr &cam) const;
    bool getTagRelPose(const RigidBodyPtr &rb, int tagId,
                       gtsam::Pose3 *pose) const;
    unsigned int  getMaxNumBodies() const;
    
    PoseEstimate getTagWorldPose(const RigidBodyConstPtr &rb,
                                 int tagId, unsigned int frame_num) const;
    bool getBodyPose(const RigidBodyConstPtr &rb, PoseEstimate *pe,
                     unsigned int frame) const;
    void printDistances() const;

    std::pair<gtsam::Point3, bool>
    getDifference(const RigidBodyPtr &rb1, const RigidBodyPtr &rb2,
                  const TagConstPtr &tag1, int corner1,
                  const TagConstPtr &tag2, int corner2) const;

    std::pair<gtsam::Point3, bool>
    getPosition(const RigidBodyPtr &rb, const TagConstPtr &tag,
                int corner) const;
    PoseEstimate getPoseEstimate(const gtsam::Symbol &sym,
                                 const gtsam::Pose3 &pose) const;
    void testProjection(const CameraConstPtr &cam, const RigidBodyPtr &rb, const TagVec &tags,
                        unsigned int frame_num);

  private:
    bool findInitialTagPose(const Tag &tag, gtsam::Pose3 *pose,
                            PoseNoise *noise) const;
    double tryOptimization(gtsam::Values *result,
                           const gtsam::ISAM2 &graph,
                           const gtsam::Values &values,
                           const std::string &verbosity, int maxIter);
    IsotropicNoisePtr             pixelNoise_;
    gtsam::Values                 values_;
    gtsam::Values                 optimizedValues_;
    //gtsam::ExpressionFactorGraph  graph_
    std::shared_ptr<gtsam::ISAM2> graph_;
    gtsam::ExpressionFactorGraph  fullGraph_;
    std::map<std::string, int>    staticObjects_;
    double                        optimizerError_{0};
    int                           optimizerIterations_{0};
    std::map<gtsam::Symbol, gtsam::Matrix> covariances_;
    gtsam::ExpressionFactorGraph  newGraph_;
    gtsam::Values                 newValues_;
  };
}

#endif
