/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/boost_graph.h"
#include "tagslam/vertex_desc.h"
#include "tagslam/optimizer.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/geometry.h"
#include "tagslam/vertex.h"
#include "tagslam/value/pose.h"
#include "tagslam/factor/tag_projection.h"
#include "tagslam/factor/absolute_pose_prior.h"
#include "tagslam/factor/relative_pose_prior.h"

#include <ros/ros.h>

#include <climits>
#include <map>
#include <unordered_map>
#include <memory>

#pragma once

namespace tagslam {
  class Graph {
    using string = std::string;
  public:
    Graph();

    void setVerbosity(const string &v) {
      optimizer_->setVerbosity(v);
    }
    inline bool hasId(const VertexId &id) const {
      return (idToVertex_.count(id) != 0); }
    bool hasPose(const ros::Time &t,
                 const std::string &name) const;
    inline bool isOptimized(const VertexDesc &v) const {
      return (graph_[v]->isOptimized()); }
    string info(const VertexDesc &v) const;
    double optimize();
    double optimizeFull(bool force = false);
    std::vector<VertexDesc> getConnected(const VertexDesc &v) const;
  
    VertexDesc add(const PoseValuePtr &p);
    VertexDesc add(const RelativePosePriorFactorPtr &p);
    VertexDesc add(const AbsolutePosePriorFactorPtr &p);
    VertexDesc add(const TagProjectionFactorPtr &p);
    
    OptimizerKey addToOptimizer(value::Pose               *p);
    OptimizerKey addToOptimizer(factor::RelativePosePrior *p);
    OptimizerKey addToOptimizer(factor::AbsolutePosePrior *p);
    OptimizerKey addToOptimizer(factor::TagProjection     *p);

    OptimizerKey addToOptimizer(const VertexDesc &v);

    VertexDesc addPose(const ros::Time &t, const string &name,
                   const Transform &pose, bool poseIsValid);

    Transform getOptimizedPose(const VertexDesc &v) const;
    // for debugging, allow to switch individual pose in opt
    void      setOptimizedPose(const VertexDesc v,
                               const Transform &pose);
    // for debugging, compute error on graph
    double    getError() { return (optimizer_->errorFull()); }
    void      plotDebug(const ros::Time &t, const string &tag);
    void      transferOptimizedPose(const VertexDesc &v);
    void      transferOptimizedValues();

    void  copyFrom(const Graph &g, const std::list<VertexDesc> &vsrc,
                   std::list<VertexDesc> *vdest);
    void  initializeFrom(const Graph &sg);
    void  print(const std::string &pre = "") const;
    std::string getStats() const;
    
    // TODO use operator [] overloading for this
    VertexPtr getVertex(const VertexDesc f) const { return (graph_[f]); }
    
    VertexDesc findPose(const ros::Time &t, const string &name) const;
  
    // static methods
    static string tag_name(int tagid);
    static string body_name(const string &body);
    static string cam_name(const string &cam);
    inline static bool is_valid(const VertexDesc &v) {
      return (v != ULONG_MAX);
    }
    static void transfer_optimized_pose(Graph *destGraph, const VertexDesc &destVertex,
                                        const Graph &srcGraph, const VertexDesc &srcVertex);

  private:
    inline VertexDesc find(const VertexId &id) const {
      const auto it = idToVertex_.find(id);
      return (it == idToVertex_.end() ? ULONG_MAX : it->second);
    }
    VertexDesc insertVertex(const VertexPtr &vp);
    std::vector<ValueKey> getOptKeysForFactor(VertexDesc fv) const;

    typedef std::unordered_map<VertexId, VertexDesc> IdToVertexMap;
    // ------ variables --------------
    BoostGraph                 graph_;
    IdToVertexMap              idToVertex_;
    std::shared_ptr<Optimizer> optimizer_;
  };
  typedef std::shared_ptr<Graph> GraphPtr;
  typedef std::shared_ptr<const Graph> GraphConstPtr;
}
