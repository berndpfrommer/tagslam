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
#include "tagslam/factor/distance.h"
#include "tagslam/factor/coordinate.h"
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
    typedef std::multimap<double, VertexDesc> ErrorToVertexMap;
    typedef std::map
    <ros::Time, std::vector<std::pair<FactorConstPtr, double>>> TimeToErrorMap;
    
    bool hasId(const VertexId &id) const { return (idToVertex_.count(id)!=0);}
    bool hasPose(const ros::Time &t, const string &name) const;
    bool isOptimized(const VertexDesc &v) const {
      return (optimized_.find(v) != optimized_.end());
    }
    bool isOptimizableFactor(const VertexDesc &v) const;

    string info(const VertexDesc &v) const;
    double optimize(double thresh);
    double optimizeFull(bool force = false);

    const VertexVec  &getFactors() const { return (factors_); }
    VertexVec getOptimizedFactors() const;
    std::vector<OptimizerKey> getOptimizerKeys(const VertexVec &vv) const;
    const BoostGraph &getBoostGraph() const { return (graph_); }

    VertexVec getConnected(const VertexDesc &v) const;

    void addEdge(const VertexDesc &from, const VertexDesc &to, int edgeId) {
      boost::add_edge(from, to, GraphEdge(edgeId), graph_);
    }
    
    VertexDesc addPose(const ros::Time &t, const string &name,
                       bool isCamPose = false);
  
    Transform getOptimizedPose(const VertexDesc &v) const;
    inline Transform pose(const VertexDesc &v) const {
      return (getOptimizedPose(v)); }
    
    PoseNoise getPoseNoise(const VertexDesc &v) const;

    PoseValueConstPtr getPoseVertex(const VertexDesc f) const {
      return (std::dynamic_pointer_cast<const value::Pose>(graph_[f]));
    }
    VertexPtr getVertex(const VertexDesc f) const { return (graph_[f]); }
    VertexPtr operator[](const VertexDesc f) const { return (graph_[f]); }
    std::pair<BoostGraph::vertex_iterator, BoostGraph::vertex_iterator>
    getVertexIterator() const { return (boost::vertices(graph_)); }
    
    void setVerbosity(const string &v) {
      optimizer_->setVerbosity(v);
    }
    void  print(const string &pre = "") const;
    string getStats() const;
    //
    // deep copy and other nasty stuff
    //
    Graph *clone() const;
    void transferFullOptimization() { optimizer_->transferFullOptimization(); }
    //
    // methods related to optimization
    //
    Optimizer *getOptimizer() const { return (optimizer_.get()); }
    std::vector<ValueKey> getOptKeysForFactor(VertexDesc fv, int nk) const;
    void markAsOptimized(const VertexDesc &v, const std::vector<FactorKey> &f);
    void markAsOptimized(const VertexDesc &v, const FactorKey &f);
    void verifyUnoptimized(const VertexDesc &v) const;
    //
    // methods for finding vertexes in the graph
    //
    VertexDesc find(const Vertex *vp) const;
    inline VertexDesc find(const VertexId &id) const {
      // inlined function for search by string
      const auto it = idToVertex_.find(id);
      return (it == idToVertex_.end() ? ULONG_MAX : it->second);
    }
    inline VertexDesc findPose(const ros::Time &t, const string &name) const {
      return (find(value::Pose::id(t, name)));
    }
    inline VertexDesc findTagPose(int tagId) const {
      return (findPose(ros::Time(0), tag_name(tagId)));
    }
    inline VertexDesc findBodyPose(const ros::Time &t, const string &n) const {
      return (findPose(t, body_name(n)));
    }
    inline VertexDesc findCameraPose(const ros::Time &t,const string &c) const{
      return (findPose(t, cam_name(c)));
    }
    VertexDesc insertVertex(const VertexPtr &vp);
    VertexDesc insertFactor(const VertexPtr &vp);
  
    // for debugging, compute error on graph
    void   printUnoptimized() const;
    void   printErrorMap(const string &prefix) const;
    double getError(const VertexDesc &v) const;
    double getError() { return (optimizer_->errorFull()); }
    double getMaxError() { return (optimizer_->getMaxError()); }
    void   plotDebug(const ros::Time &t, const string &tag);
    ErrorToVertexMap getErrorMap() const;
    TimeToErrorMap   getTimeToErrorMap() const;
    // static methods
    static string tag_name(int tagid);
    static string body_name(const string &body);
    static string cam_name(const string &cam);
    inline static bool is_valid(const VertexDesc &v) {
      return (v != ULONG_MAX);
    }
  private:
    typedef std::unordered_map<VertexId, VertexDesc> IdToVertexMap;
    typedef std::unordered_map<VertexDesc,
                               std::vector<OptimizerKey>> VertexToOptMap;
    VertexToOptMap::const_iterator findOptimized(const VertexDesc &v) const;
    ValueKey findOptimizedPoseKey(const VertexDesc &v) const;

    // ------ variables --------------
    BoostGraph                 graph_;
    VertexVec                  factors_;
    IdToVertexMap              idToVertex_;
    VertexToOptMap             optimized_;
    std::shared_ptr<Optimizer> optimizer_;
  };
  typedef std::shared_ptr<Graph> GraphPtr;
  typedef std::shared_ptr<const Graph> GraphConstPtr;
}
