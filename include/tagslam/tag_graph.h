/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_TAG_GRAPH_H
#define TAGSLAM_TAG_GRAPH_H

#include "tagslam/utils.h"
#include "tagslam/tag.h"
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
    
    void addTags(const std::string &objectName,
                 const gtsam::Pose3 &objPose,
                 const utils::PoseNoise &objPoseNoise,
                 const std::vector<Tag> &tags);
    void observedTags(int cam_idx, const std::vector<Tag> &tags,
                      unsigned int frame_num,
                      const gtsam::Pose3 &cam_init_pose);

    void addCamera(int cam_idx,
                   const std::vector<double> &intr,
                   const std::string &distModel,
                   const std::vector<double> &distCoeff);
    bool hasStaticObject(const std::string &name) const;
    bool getCameraPose(int cam_idx, gtsam::Pose3 *pose) const;

    void getCameraPoses(std::vector<std::pair<int, gtsam::Pose3>> *poses,
                        unsigned int frame_num) const;
    gtsam::Pose3 getTagWorldPose(int tagId) const;
    void getTagWorldPoses(std::vector<std::pair<int, gtsam::Pose3>> *poses) const;

  private:
    struct GraphCam {
      GraphCam(const boost::shared_ptr<gtsam::Cal3DS2> &m =
               boost::shared_ptr<gtsam::Cal3DS2>(),
               const std::vector<double> &i = std::vector<double>(),
               const std::string &dm  = "radtan",
               const std::vector<double> &dc = std::vector<double>()) :
        gtsamCameraModel(m), intrinsics(i), distModel(dm), distCoeff(dc) {
      };
      boost::shared_ptr<gtsam::Cal3DS2> gtsamCameraModel;
      std::vector<double> intrinsics;
      std::string         distModel;
      std::vector<double> distCoeff;
      gtsam::Pose3        lastPose;
      bool                hasValidPose{false};
    };
    void optimize();
    bool getPoints(std::vector<cv::Point2f> *img_pts,
                   std::vector<cv::Point3f> *world_pts,
                   const std::vector<Tag> &tags) const;
    bool findInitialTagPose(const Tag &tag, gtsam::Pose3 *pose,
                            Tag::PoseNoise *noise) const;
    void updateCameraPoses(unsigned int frameNum);
    gtsam::Point3 insertTagType(const Tag &tag, int corner);

    double tryOptimization(gtsam::Values *result,
                           const gtsam::NonlinearFactorGraph &graph,
                           const gtsam::Values &values,
                           const std::string &verbosity, int maxIter);
#if 0    
    void addStaticObject(const std::string &objectName, const gtsam::Pose3 &pose,
                         const utils::PoseNoise &poseNoise);
#endif    

    typedef std::map<int, GraphCam> CamMap;
    gtsam::Values                 values_;
    gtsam::Values                 optimizedValues_;
    gtsam::NonlinearFactorGraph   graph_;
    std::map<std::string, int>    staticObjects_;
    CamMap                        cameras_;
    double                        optimizerError_{0};
    int                           optimizerIterations_{0};
  };
}

#endif
