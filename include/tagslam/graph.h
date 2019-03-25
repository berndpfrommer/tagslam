/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/boost_graph.h"
#include "tagslam/optimizer.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/geometry.h"
#include "tagslam/tag2.h"
#include "tagslam/camera2.h"
#include "tagslam/vertex.h"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <climits>
#include <map>
#include <unordered_map>
#include <memory>

#pragma once

namespace tagslam {
  class Graph {
    using string = std::string;
  public:
    typedef std::string Id;
    typedef BoostGraphVertex Vertex;
    
    Graph();

    inline bool hasId(const Id &id) const {
      return (idToVertex_.count(id) != 0); }
    inline bool hasPose(const ros::Time &t,
                        const std::string &name) const {
      return (hasId(make_id(t, name))); }
    inline bool isOptimized(const Vertex &v) const {
      return (graph_[v].vertex->isOptimized()); }
    string info(Vertex v) const;
    void   optimize();
    void   optimizeFull(bool force = false);
    std::vector<Vertex> getConnected(const Vertex &v) const;
    Vertex addPose(const ros::Time &t, const string &name,
                   const Transform &pose, bool poseIsValid);
    void   addPoseToOptimizer(const Graph::Vertex &v);
    Vertex addAbsolutePosePriorFactor(const ros::Time &t, const string &name,
                                      const PoseWithNoise &pn);
    void   addAbsolutePosePriorFactorToOptimizer(const Graph::Vertex &v);
    Vertex addRelativePosePriorFactor(const ros::Time &tPrev, const ros::Time &tCurr,
                                      const std::string &name,
                                      const PoseWithNoise &deltaPose);
    void   addRelativePosePriorFactorToOptimizer(const Graph::Vertex &v);
    Vertex addTagProjectionFactor(const ros::Time &t,
                                  const Tag2ConstPtr &tag,
                                  const Camera2ConstPtr &cam,
                                  double noise,
                                  const geometry_msgs::Point *imgCorners);
    void   addTagProjectionFactorToOptimizer(const Graph::Vertex &v);
    Transform getOptimizedPose(const Vertex &v) const;
    void      plotDebug(const ros::Time &t, const string &tag);
    void      transferOptimizedValues();
    
    void      addToOptimizer(const Graph::Vertex &gv);
    // use operator overloading for this
    VertexPtr getVertex(const Vertex f) { return (graph_[f].vertex); }
    
    inline Vertex find(const ros::Time &t, const string &name) const {
      return (find(make_id(t, name)));
    }
    static Graph::Id make_id(const ros::Time &t, const std::string &name);

    // static methods
    static string tag_name(int tagid);
    static string body_name(const string &body);
    static string cam_name(const string &cam);
    inline static bool is_valid(const Vertex &v) {
      return (v != ULONG_MAX);
    }

  private:
    inline Vertex find(const Id &id) const {
      const auto it = idToVertex_.find(id);
      return (it == idToVertex_.end() ? ULONG_MAX : it->second);
    }
    Vertex addVertex(const VertexPtr &vp, const std::string &id);
    std::vector<ValueKey> getOptKeysForFactor(Graph::Vertex fv) const;

    typedef std::unordered_map<Id, Vertex> IdToVertexMap;
    // ------ variables --------------
    BoostGraph                 graph_;
    IdToVertexMap              idToVertex_;
    std::shared_ptr<Optimizer> optimizer_;
  };
}
