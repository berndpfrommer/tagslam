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

namespace tagslam {
  class TagGraph {
    typedef gtsam::noiseModel::Isotropic::shared_ptr	IsotropicNoisePtr;
  public:
    TagGraph() {};
    virtual ~TagGraph() {};
    TagGraph(const TagGraph&) = delete;
    TagGraph& operator=(const TagGraph&) = delete;

    void addTags(const std::string &objectName,
                 const gtsam::Pose3 &objPose,
                 const utils::PoseNoise &objPoseNoise,
                 const std::vector<Tag> &tags);
    void observedTags(int cam_idx, const std::vector<Tag> &tags,
                      unsigned int frame_num, const cv::Mat &K,
                      const cv::Mat &D);

    void addCamera(int cam_idx, double fx, double fy,
                   double cx, double cy,
                   const std::string &distModel,
                   const std::vector<double> &distCoeff);
    bool hasStaticObject(const std::string &name) const;
    void getCameraPoses(std::vector<std::pair<int, gtsam::Pose3>> *poses,
                        unsigned int frame_num) const;
    void getTagPoses(std::vector<std::pair<int, gtsam::Pose3>> *poses) const;

  private:
    void optimize();
    void getPoints(std::vector<cv::Point2f> *img_pts,
                   std::vector<cv::Point3f> *world_pts,
                   const std::vector<Tag> &tags) const;
#if 0    
    void addStaticObject(const std::string &objectName, const gtsam::Pose3 &pose,
                         const utils::PoseNoise &poseNoise);
#endif    

    typedef std::map<int, boost::shared_ptr<gtsam::Cal3DS2> > CamMap;
    gtsam::Point3 insertTagType(const Tag &tag, int corner);
    gtsam::Values                 values_;
    gtsam::Values                 optimizedValues_;
    gtsam::NonlinearFactorGraph   graph_;
    std::map<std::string, int>    staticObjects_;
    CamMap                        cameras_;
  };
}

#endif
