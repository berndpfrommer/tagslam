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
    GraphManager();
    ~GraphManager();
    // --------------------- good
    void setPixelNoise(double pn) { pixelNoise_ = pn; }
    void setOptimizeFullGraph(bool fg) { optimizeFullGraph_ = fg; }
    void setMaxSubgraphError(double e) { maxSubgraphError_ = e; }
    void setAngleLimit(double angDeg);

    double optimize(double thresh);
    double reoptimize();
    VertexDesc addPose(const ros::Time &t, const string &name,
                       const Transform &pose, bool poseIsValid,
                       bool isCamPose = false);
    VertexDesc addPoseWithPrior(const ros::Time &t, const string &name,
                                const PoseWithNoise &pn, bool isCamPose = false);
    VertexDesc addTagProjectionFactor(const ros::Time &t,
                                         const Tag2ConstPtr &tag,
                                         const Camera2ConstPtr &cam,
                                         const geometry_msgs::Point *imgCorners);

    void plotDebug(const ros::Time &t, const string &tag);
    void addBody(const Body &body);
    bool getPose(const ros::Time &t, const string &id, Transform *tf) const;
    PoseWithNoise getCameraPoseWithNoise(const Camera2ConstPtr &cam) const;

    void addTag(const Tag2 &tag);
    void processNewFactors(const ros::Time &t,
                           const std::vector<VertexDesc> &facs);
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
    struct SubGraph {
      typedef std::deque<VertexDesc> FactorCollection;
      typedef std::set<VertexDesc>  ValueCollection;
      FactorCollection  factors;
      ValueCollection   values;
      double            error_{0};
    };

    typedef std::map<ros::Time, std::vector<VertexDesc>> TimeToVertexesMap;

    
    
    VertexDesc addPrior(const ros::Time &t,
                           const string &name,
                           const PoseWithNoise &pn);
    void examine(const ros::Time &t, VertexDesc fac,
                 std::deque<VertexDesc> *factorsToExamine,
                 SubGraph *found, SubGraph *sg);
    std::vector<std::deque<VertexDesc>>
    findSubgraphs(const ros::Time &t,
                  const std::vector<VertexDesc> &fac,
                  SubGraph *found);
    void initializeSubgraphs(
          std::vector<GraphPtr> *subGraphs,
          const std::vector<std::deque<VertexDesc>> &verts);
    void exploreSubGraph(const ros::Time &t,
                         VertexDesc start,
                         SubGraph *subGraph, SubGraph *found);
    double optimizeSubgraphs(const std::vector<GraphPtr> &subGraphs);
    double initializeFromSubgraphs(const std::vector<GraphPtr> &subGraphs);

    
    // ------ variables --------------
    double             pixelNoise_{1.0};
    bool               optimizeFullGraph_;
    Graph              graph_;
    TimeToVertexesMap  times_;
    Profiler           profiler_;
    size_t             numNoFactors_{0};
    int                numIncrementalOpt_{0};
    int                maxNumIncrementalOpt_{100};
    double             maxSubgraphError_{15.0};
    double             subgraphError_{0};
    double             angleLimit_{0};
  };
}
