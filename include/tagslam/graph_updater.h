/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph.h"
#include "tagslam/value/pose.h"
#include "tagslam/profiler.h"
#include "tagslam/init_pose.h"

#include <ros/ros.h>
#include <memory>
#include <set>
#include <deque>

#pragma once

namespace tagslam {
  class Optimizer;
  namespace value {
    class Pose;
  }
 
  struct SubGraph {
    typedef std::deque<VertexDesc> FactorCollection;
    typedef std::set<VertexDesc>  ValueCollection;
    FactorCollection  factors;
    ValueCollection   values;
    double            error_{0};
  };

  class GraphUpdater {
    using string = std::string;
  public:
    typedef std::deque<VertexDesc> VertexDeque;
    // ---------------------
    void setOptimizerMode(const std::string &mode);
    void processNewFactors(Graph *g,
                           const ros::Time &t, const VertexVec &facs);
    void printPerformance();
    double getPixelNoise() const { return (pixelNoise_); }
    const string &getOptimizerMode() const { return (optimizerMode_); }
    void parse(XmlRpc::XmlRpcValue config);
  private:
    typedef std::map<ros::Time, VertexVec> TimeToVertexesMap;
    void examine(Graph *graph, const ros::Time &t, VertexDesc fac,
                 VertexDeque *factorsToExamine,
                 SubGraph *found, SubGraph *sg);

    std::vector<VertexDeque>
    findSubgraphs(Graph *g, const ros::Time &t, const VertexVec &fac,
                  SubGraph *found);
 
    double initializeSubgraphs(Graph *g, std::vector<GraphPtr> *subGraphs,
                               const std::vector<VertexDeque> &verts);
    void exploreSubGraph(Graph *g, const ros::Time &t, VertexDesc start,
                         SubGraph *subGraph, SubGraph *found);
    bool applyFactorsToGraph(Graph *g, const ros::Time &t,
                             const VertexVec &facs, SubGraph *covered);
    void eraseStoredFactors(const ros::Time &t,
                            const SubGraph::FactorCollection &covered);
    double optimize(Graph *g, double thresh);
    // ------ variables --------------
    TimeToVertexesMap  oldFactors_;
    Profiler           profiler_;
    int                numIncrementalOpt_{0};
    double             subgraphError_{0};
    double             lastIncError_{0};
    bool               optimizeFullGraph_{false};

    string             optimizerMode_{"slow"};
    double             maxSubgraphError_{15.0};
    double             subGraphAbsPriorPositionNoise_{0.001};
    double             subGraphAbsPriorRotationNoise_{0.001};
    double             pixelNoise_{0.001};
    int                maxNumIncrementalOpt_{100};
    init_pose::Params  poseInitParams_;
    double             minimumViewingAngle_{0};
    double             maxAmbiguityRation_{0.3};
    double             ambiguityAngleThreshold_{1.0};
  };
}
