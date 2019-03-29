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
    double optimize();
    double reoptimize();
    VertexDesc addPose(const ros::Time &t, const string &name,
                          const Transform &pose, bool poseIsValid);
    VertexDesc addPoseWithPrior(const ros::Time &t, const string &name,
                                   const PoseWithNoise &pn);
    VertexDesc addTagProjectionFactor(const ros::Time &t,
                                         const Tag2ConstPtr &tag,
                                         const Camera2ConstPtr &cam,
                                         const geometry_msgs::Point *imgCorners);

    void plotDebug(const ros::Time &t, const string &tag);
    void addBody(const Body &body);
    bool getPose(const ros::Time &t, const string &id, Transform *tf) const;
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
      typedef std::list<VertexDesc> FactorCollection;
      typedef std::set<VertexDesc>  ValueCollection;
      FactorCollection  factors;
      ValueCollection   values;
    };

    typedef std::map<ros::Time, std::vector<VertexDesc>> TimeToVertexesMap;

    
    
    VertexDesc addPrior(const ros::Time &t,
                           const string &name,
                           const PoseWithNoise &pn);
    void examine(const ros::Time &t, VertexDesc fac,
                 std::list<VertexDesc> *factorsToExamine,
                 SubGraph *found, SubGraph *sg);
    std::vector<std::list<VertexDesc>>
    findSubgraphs(const ros::Time &t,
                  const std::vector<VertexDesc> &fac,
                  SubGraph *found);
    void exploreSubGraph(const ros::Time &t,
                         VertexDesc start,
                         SubGraph *subGraph, SubGraph *found);
    int findConnectedPoses(const Graph &graph,
                           VertexDesc v,
                           std::vector<PoseValuePtr> *poses,
                           std::vector<VertexDesc> *conn);
    void setValueFromTagProjection(Graph *graph,
                                   VertexDesc v, const Transform &T_c_o);
    int  setValueFromRelativePosePrior(Graph *graph,
                                       VertexDesc v, const Transform &deltaPose);
    void initializeSubgraphs(
          std::vector<GraphPtr> *subGraphs,
          const std::vector<std::list<VertexDesc>> &verts);
    void optimizeSubgraphs(const std::vector<GraphPtr> &subGraphs);
    void initializeFromSubgraphs(const std::vector<GraphPtr> &subGraphs);

    
    // ------ variables --------------
    double             pixelNoise_{1.0};
    bool               optimizeFullGraph_;
    Graph              graph_;
    TimeToVertexesMap  times_;
    Profiler           profiler_;
    size_t             numNoFactors_{0};
  };
}
