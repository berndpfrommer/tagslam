/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph.h"
#include "tagslam/body.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/value_key.h"
#include "tagslam/camera2.h"
#include "tagslam/value/pose.h"
#include "tagslam/profiler.h"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>
#include <memory>
#include <set>

#pragma once

namespace tagslam {
  class Optimizer;
  namespace value {
    class Pose;
  }
  class GraphManager {
    using string = std::string;
  public:
    void setGraph(const GraphPtr &g) { graph_ = g; }
    // --------------------- good
    void setPixelNoise(double pn) { pixelNoise_ = pn; }

    VertexDesc addPose(const ros::Time &t, const string &name,
                       bool isCamPose = false);
    VertexDesc addPoseWithPrior(const ros::Time &t, const string &name,
                                const PoseWithNoise &pn, bool isCamPose = false);
    VertexDesc addTagProjectionFactor(const ros::Time &t,
                                         const Tag2ConstPtr &tag,
                                         const Camera2ConstPtr &cam,
                                         const geometry_msgs::Point *imgCorners);
    VertexDesc addRelativePosePrior(const  RelativePosePriorFactorPtr &fac);

    void addBody(const Body &body);
    bool getPose(const ros::Time &t, const string &id, Transform *tf) const;
    PoseWithNoise getCameraPoseWithNoise(const Camera2ConstPtr &cam) const;

    void addTag(const Tag2 &tag);
    VertexDesc
    addProjectionFactor(const ros::Time &t,
                        const Tag2ConstPtr &tag,
                        const Camera2ConstPtr &cam,
                        const geometry_msgs::Point *imgCorners);
    VertexDesc
    addBodyPoseDelta(const ros::Time &tPrev, const ros::Time &tCurr,
                     const BodyConstPtr &body,
                     const PoseWithNoise &deltaPose);
  private:
    VertexDesc addPrior(const ros::Time &t,
                        const string &name,
                        const PoseWithNoise &pn);
    
    // ------ variables --------------
    double             pixelNoise_{1.0};
    GraphPtr           graph_;
  };
}
