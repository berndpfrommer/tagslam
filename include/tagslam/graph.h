/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/boost_graph.h"
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
  class Graph {
    using string = std::string;
  public:
    typedef std::string Id;
    struct VertexPose {
      VertexPose(const BoostGraphVertex &v = BoostGraphVertex(),
                 const std::shared_ptr<value::Pose> &p =
                 std::shared_ptr<value::Pose>()) : vertex(v), pose(p) {}
      BoostGraphVertex             vertex;
      std::shared_ptr<value::Pose> pose;
    };
    Graph();
    ~Graph();
    void setOptimizer(Optimizer *opt) { optimizer_ = opt; }
    void setPixelNoise(double pn) { pixelNoise_ = pn; }
    inline bool hasId(const Id &id) const {
      return (idToVertex_.count(id) != 0);
    }
    
    VertexPose findPose(const ros::Time &t, const string &name) const;
    VertexPose addPose(const ros::Time &t, const string &name,
                           const Transform &pose, bool poseIsValid);
    BoostGraphVertex addPrior(const ros::Time &t, const VertexPose &vp,
                              const string &name,
                              const PoseWithNoise &pn);
    VertexPose addPoseWithPrior(const ros::Time &t, const string &name,
                                const PoseWithNoise &pn);
    bool getPose(const ros::Time &t, const string &id, Transform *tf) const;
    void addBody(const Body &body);
    BoostGraphVertex
    addRelativePosePrior(const ros::Time &t,
                         const string &name,
                         const VertexPose &vp1,
                         const VertexPose &vp2,
                         const PoseWithNoise &deltaPose,
                         bool addToOptimizer);

    VertexPose addTag(const Tag2 &tag);
    BoostGraphVertex
    addBodyPoseDelta(const ros::Time &tPrev, const ros::Time &tCurr,
                     const BodyConstPtr &body,
                     const PoseWithNoise &deltaPose);
    void setOptimizeFullGraph(bool fg) { optimizeFullGraph_ = fg; }
    void plotDebug(const ros::Time &t, const string &tag);
    void optimize();
    void test();
    void addTagMeasurements(const BodyVec &bodies,
                            const std::vector<TagArrayConstPtr> &tagMsgs,
                            const Camera2Vec &cameras);
    BoostGraphVertex
    addProjectionFactor(const ros::Time &t,
                        const Tag2ConstPtr &tag,
                        const Camera2ConstPtr &cam,
                        const geometry_msgs::Point *imgCorners);

    static string tag_name(int tagid);
    static string body_name(const string &body);
    static string cam_name(const string &cam);
    void processNewFactors(const ros::Time &t,
                           const std::vector<BoostGraphVertex> &facs);
  private:
    struct SubGraph {
      typedef std::list<BoostGraphVertex> FactorCollection;
      typedef std::set<BoostGraphVertex>  ValueCollection;
      FactorCollection  factors;
      ValueCollection   values;
    };
    void initializeSubgraphs(const std::vector<std::list<BoostGraphVertex>> &verts);
    std::vector<std::list<BoostGraphVertex>>
    findSubgraphs(const ros::Time &t,
                  const std::vector<BoostGraphVertex> &fac,
                  SubGraph *found);

    void transferValues();
    void exploreSubGraph(const ros::Time &t,
                         BoostGraphVertex start,
                         SubGraph *subGraph, SubGraph *found);
    void examine(const ros::Time &t, BoostGraphVertex fac,
                 std::list<BoostGraphVertex> *factorsToExamine,
                 SubGraph *found, SubGraph *sg);
    void addProjectionFactorToOptimizer(const BoostGraphVertex v);
    int findConnectedPoses(BoostGraphVertex v,
                           std::vector<PoseValuePtr> *poses);
    int  setValueFromRelativePosePrior(BoostGraphVertex v, const Transform &deltaPose,
                                       std::vector<PoseValuePtr> *poses);
    void setValueFromTagProjection(BoostGraphVertex v, const Transform &T_c_o);
    std::string info(BoostGraphVertex v) const;
    typedef std::unordered_map<Id, BoostGraphVertex> IdToVertexMap;
    typedef std::map<ros::Time, std::vector<BoostGraphVertex>> TimeToVertexesMap;
    // ------ variables --------------
    BoostGraph         graph_;
    double             pixelNoise_{1.0};
    bool               optimizeFullGraph_;
    Optimizer         *optimizer_;
    IdToVertexMap      idToVertex_;
    TimeToVertexesMap  times_;
    Profiler           profiler_;
  };
}
