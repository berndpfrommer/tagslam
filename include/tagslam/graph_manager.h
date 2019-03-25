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
    void optimize();
    void reoptimize();
    Graph::Vertex addPose(const ros::Time &t, const string &name,
                          const Transform &pose, bool poseIsValid);
    Graph::Vertex addPoseWithPrior(const ros::Time &t, const string &name,
                                   const PoseWithNoise &pn);
    Graph::Vertex addTagProjectionFactor(const ros::Time &t,
                                         const Tag2ConstPtr &tag,
                                         const Camera2ConstPtr &cam,
                                         const geometry_msgs::Point *imgCorners);

    void plotDebug(const ros::Time &t, const string &tag);
    void addBody(const Body &body);
    bool getPose(const ros::Time &t, const string &id, Transform *tf) const;
    void addTag(const Tag2 &tag);
    void processNewFactors(const ros::Time &t,
                           const std::vector<BoostGraphVertex> &facs);
    Graph::Vertex
    addProjectionFactor(const ros::Time &t,
                        const Tag2ConstPtr &tag,
                        const Camera2ConstPtr &cam,
                        const geometry_msgs::Point *imgCorners);
    Graph::Vertex
    addBodyPoseDelta(const ros::Time &tPrev, const ros::Time &tCurr,
                     const BodyConstPtr &body,
                     const PoseWithNoise &deltaPose);
  private:
    struct SubGraph {
      typedef std::list<Graph::Vertex> FactorCollection;
      typedef std::set<Graph::Vertex>  ValueCollection;
      FactorCollection  factors;
      ValueCollection   values;
    };

    typedef std::map<ros::Time, std::vector<Graph::Vertex>> TimeToVertexesMap;

    
    
    Graph::Vertex addPrior(const ros::Time &t,
                           const string &name,
                           const PoseWithNoise &pn);
    void examine(const ros::Time &t, Graph::Vertex fac,
                 std::list<Graph::Vertex> *factorsToExamine,
                 SubGraph *found, SubGraph *sg);
    std::vector<std::list<Graph::Vertex>>
    findSubgraphs(const ros::Time &t,
                  const std::vector<Graph::Vertex> &fac,
                  SubGraph *found);
    void exploreSubGraph(const ros::Time &t,
                         Graph::Vertex start,
                         SubGraph *subGraph, SubGraph *found);
    int findConnectedPoses(Graph::Vertex v,
                           std::vector<PoseValuePtr> *poses,
                           std::vector<Graph::Vertex> *conn);
    void setValueFromTagProjection(Graph::Vertex v, const Transform &T_c_o);
    int  setValueFromRelativePosePrior(Graph::Vertex v, const Transform &deltaPose);
    void initializeSubgraphs(const std::vector<std::list<Graph::Vertex>> &verts);

    
    // ------ variables --------------
    double             pixelNoise_{1.0};
    bool               optimizeFullGraph_;
    Graph              graph_;
    TimeToVertexesMap  times_;
    Profiler           profiler_;
  };
}
