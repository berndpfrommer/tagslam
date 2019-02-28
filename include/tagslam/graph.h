/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/boost_graph.h"
#include "tagslam/body.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/value_key.h"
#include "tagslam/camera2.h"

#include <ros/ros.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>
#include <memory>

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
    ~Graph() {};
    void setOptimizer(Optimizer *opt) { optimizer_ = opt; }
    
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
    void addBodyPoseDelta(const ros::Time &tPrev, const ros::Time &tCurr,
                          const BodyConstPtr &body,
                          const PoseWithNoise &deltaPose);
    void setOptimizeFullGraph(bool fg) { optimizeFullGraph_ = fg; }
    void plotDebug(const ros::Time &t, const string &tag);
    void optimize();
    void test();
    void addTagMeasurements(const BodyVec &bodies,
                            const std::vector<TagArrayConstPtr> &tagMsgs,
                            const Camera2Vec &cameras);
    inline bool hasId(const Id &id) const {
      return (idToVertex_.count(id) != 0);
    }

    static string tag_name(int tagid);
    static string body_name(const string &body);
    static string cam_name(const string &cam);
  private:
    typedef std::unordered_map<Id, BoostGraphVertex> IdToVertexMap;
    // ------ variables --------------
    BoostGraph         graph_;
    bool               optimizeFullGraph_;
    Optimizer         *optimizer_;
    IdToVertexMap      idToVertex_;
  };
}
