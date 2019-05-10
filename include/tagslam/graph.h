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
    bool hasPose(const ros::Time &t, const std::string &name) const;
    bool isOptimized(const VertexDesc &v) const {
      return (optimized_.find(v) != optimized_.end());
    }
    string info(const VertexDesc &v) const;
    double optimize(double thresh);
    double optimizeFull(bool force = false);

    const VertexVec &getFactors() const { return (factors_); }
    const BoostGraph &getBoostGraph() const { return (graph_); }

    VertexVec getConnected(const VertexDesc &v) const;
    bool isOptimizableFactor(const VertexDesc &v) const;

    VertexDesc add(const PoseValuePtr &p);
    VertexDesc add(const RelativePosePriorFactorPtr &p);
    VertexDesc add(const AbsolutePosePriorFactorPtr &p);
    VertexDesc add(const DistanceFactorPtr &p);
    VertexDesc add(const CoordinateFactorPtr &p);
    VertexDesc add(const TagProjectionFactorPtr &p);
    
    OptimizerKey addToOptimizer(const VertexDesc &v, const Transform &tf);
    OptimizerKey addToOptimizer(const factor::RelativePosePrior *p);
    OptimizerKey addToOptimizer(const factor::AbsolutePosePrior *p);
    OptimizerKey addToOptimizer(const factor::Distance *p);
    OptimizerKey addToOptimizer(const factor::Coordinate *p);
   std::vector<OptimizerKey> addToOptimizer(const factor::TagProjection *p);

    VertexDesc addPose(const ros::Time &t, const string &name,
                       bool isCamPose = false);
  
    Transform getOptimizedPose(const VertexDesc &v) const;
    inline Transform pose(const VertexDesc &v) const {
      return (getOptimizedPose(v)); }
    double getOptimizedDistance(const VertexDesc &v) const;
    double getOptimizedCoordinate(const VertexDesc &v) const;
    
    PoseNoise2 getPoseNoise(const VertexDesc &v) const;

    void      transferFullOptimization() {
      optimizer_->transferFullOptimization(); }
    void setVerbosity(const string &v) {
      optimizer_->setVerbosity(v);
    }
    void  copyFrom(const Graph &g, const std::deque<VertexDesc> &vsrc);
    void  initializeFrom(const Graph &sg);
    void  print(const std::string &pre = "") const;
    std::string getStats() const;
    
    VertexPtr getVertex(const VertexDesc f) const { return (graph_[f]); }
    VertexPtr operator[](const VertexDesc f) const { return (graph_[f]); }

    VertexDesc findPose(const ros::Time &t, const string &name) const;
    // for debugging, compute error on graph
    void printUnoptimized() const;
    void printErrorMap(const std::string &prefix) const;
    double getError(const VertexDesc &v) const;
    double getError() { return (optimizer_->errorFull()); }
    double getMaxError() { return (optimizer_->getMaxError()); }
    void   plotDebug(const ros::Time &t, const string &tag);
    ErrorToVertexMap getErrorMap() const;
    TimeToErrorMap getTimeToErrorMap() const;
    // static methods
    static string tag_name(int tagid);
    static string body_name(const string &body);
    static string cam_name(const string &cam);
    static string dist_name(const string &dist);
    inline static bool is_valid(const VertexDesc &v) {
      return (v != ULONG_MAX);
    }
  private:
    typedef std::unordered_map<VertexId, VertexDesc> IdToVertexMap;
    typedef std::unordered_map<VertexDesc,
                               std::vector<OptimizerKey>> VertexToOptMap;
    inline VertexDesc find(const VertexId &id) const {
      const auto it = idToVertex_.find(id);
      return (it == idToVertex_.end() ? ULONG_MAX : it->second);
    }
    VertexDesc find(const Vertex *vp) const;
    VertexToOptMap::const_iterator findOptimized(const VertexDesc &v) const;
    void verifyUnoptimized(const VertexDesc &v) const;
    ValueKey findOptimizedPoseKey(const VertexDesc &v) const;
    VertexDesc insertVertex(const VertexPtr &vp);
    std::vector<ValueKey> getOptKeysForFactor(VertexDesc fv, int nk) const;

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
