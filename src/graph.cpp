/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph.h"
#include "tagslam/body.h"
#include "tagslam/logging.h"
#include "tagslam/camera.h"
#include "tagslam/gtsam_optimizer.h"
#include "tagslam/value/pose.h"
#include "tagslam/factor/absolute_pose_prior.h"
#include "tagslam/factor/relative_pose_prior.h"
#include "tagslam/factor/tag_projection.h"
#include <boost/range/irange.hpp>


#include <sstream>

namespace tagslam {

  using boost::irange;
  using std::string;

  Graph::Graph() {
    optimizer_.reset(new GTSAMOptimizer());
  }

  Graph *Graph::clone() const {
    // it's not really a full deep copy, because the vertex pointers
    // are copied shallow. 
    Graph *g = new Graph(*this);
    // The optimizer is just a pointer, need to clone it
    g->optimizer_.reset(optimizer_->clone());
    return (g);
  }

  double Graph::optimize(double thresh) {
    return (optimizer_->optimize(thresh));
  }
  
  double Graph::optimizeFull(bool force) {
    return (optimizer_->optimizeFull(force));
  }

  VertexDesc
  Graph::insertVertex(const VertexPtr &vp) {
    const VertexDesc nv = boost::add_vertex(GraphVertex(vp), graph_);
    idToVertex_.insert(IdToVertexMap::value_type(vp->getId(), nv));
    return (nv);
  }
  
  VertexDesc
  Graph::insertFactor(const VertexPtr &vp) {
    const VertexDesc nv = insertVertex(vp);
    factors_.push_back(nv);
    return (nv);
  }

  std::vector<VertexDesc>
  Graph::getConnected(const VertexDesc &v) const {
    auto edges = boost::out_edges(v, graph_);
    std::vector<VertexDesc> c;
    for (auto edgeIt = edges.first; edgeIt != edges.second; ++edgeIt) {
      c.push_back(boost::target(*edgeIt, graph_));
    }
    return (c);
  }

  bool
  Graph::isOptimizableFactor(const VertexDesc &v) const {
    if (graph_[v]->isValue()) {
      BOMB_OUT("vertex is no factor: " << graph_[v]->getLabel());
    }
    for (const auto &vv: getConnected(v)) {
      if (!isOptimized(vv)) {
        return (false);
      }
    }
    return (true);
  }

  std::vector<ValueKey>
  Graph::getOptKeysForFactor(VertexDesc fv, int numKeys) const {
    auto edges = boost::out_edges(fv, graph_);
    std::vector<ValueKey> optKeys;
    for (auto edgeIt = edges.first; edgeIt != edges.second; ++edgeIt) {
      VertexDesc vv  = boost::target(*edgeIt, graph_); // value vertex
      VertexPtr     vvp = graph_[vv]; // pointer to value
      ValuePtr       vp = std::dynamic_pointer_cast<value::Value>(vvp);
      if (!vp) {
        BOMB_OUT("vertex is no pose: " << vvp->getLabel());
      }
      optKeys.push_back(findOptimizedPoseKey(vv));
    }
    if (optKeys.size() != (size_t)numKeys) {
      BOMB_OUT("wrong num values for " << info(fv) << ": "
               << optKeys.size() << " expected: " << numKeys);
    }

    return (optKeys);
  }


  VertexDesc Graph::find(const Vertex *vp) const {
    VertexDesc v = find(vp->getId());
    if (!is_valid(v)) {
      BOMB_OUT("cannot find factor " << vp->getLabel());
    }
    return (v);
  }

  Graph::VertexToOptMap::const_iterator
  Graph::findOptimized(const VertexDesc &v) const {
    VertexToOptMap::const_iterator it = optimized_.find(v);
    if (it == optimized_.end()) {
      BOMB_OUT("not optimized: " << info(v));
    }
    return (it);
  }

  void Graph::verifyUnoptimized(const VertexDesc &v) const {
    const VertexToOptMap::const_iterator it = optimized_.find(v);
    if (it != optimized_.end()) {
      BOMB_OUT("already optimized: " << info(v));
    }
  }

  ValueKey Graph::findOptimizedPoseKey(const VertexDesc &v) const {
    VertexToOptMap::const_iterator it = optimized_.find(v);
    if (it == optimized_.end()) {
      BOMB_OUT("cannot find opt pose: " << info(v));
    }
    if (it->second.size() != 1) {
      BOMB_OUT("pose must have one opt value: " << info(v)
               << " but has: " << it->second.size());
    }
    return (it->second[0]);
  }
  
  void Graph::markAsOptimized(const VertexDesc &v,
                              const std::vector<FactorKey> &f) {
    optimized_.insert(VertexToOptMap::value_type(v, f));
  }
 
  void Graph::markAsOptimized(const VertexDesc &v, const FactorKey &fk) {
    optimized_.insert(
      VertexToOptMap::value_type(v, std::vector<FactorKey>(1, fk)));
  }


  VertexDesc
  Graph::addPose(const ros::Time &t, const string &name, bool isCameraPose) {
    if (hasId(value::Pose::id(t, name))) {
      BOMB_OUT("duplicate pose inserted: " << t << " " << name);
    }
    PoseValuePtr pv(new value::Pose(t, name, isCameraPose));
    return (insertVertex(pv));
  }


  Transform Graph::getOptimizedPose(const VertexDesc &v) const {
    PoseValueConstPtr vp = std::dynamic_pointer_cast<value::Pose>(graph_[v]);
    if (!vp) {
      BOMB_OUT("vertex is not pose: " << info(v));
    }
    VertexToOptMap::const_iterator it = findOptimized(v);
    return (optimizer_->getPose(it->second[0]));
  }


  string Graph::info(const VertexDesc &v) const {
    return (graph_[v]->getLabel());
  }

  bool Graph::hasPose(const ros::Time &t,
                      const string &name) const {
    return (hasId(value::Pose::id(t, name)));
  }


  void Graph::print(const string &prefix) const {
    for (auto v = boost::vertices(graph_); v.first != v.second; ++v.first) {
      bool isOpt = (optimized_.find(*v.first) != optimized_.end());
      ROS_DEBUG_STREAM(prefix << " " << graph_[*v.first]->getLabel()
                       << ":" << (isOpt ? "O":"U"));
      if (graph_[*v.first]->isValue()) {
        PoseValueConstPtr  vp = std::dynamic_pointer_cast<const value::Pose>(graph_[*v.first]);
      }
    }
  }

  string Graph::getStats() const {
    int numFac(0), numOptFac(0), numVal(0), numOptVal(0);
    for (auto v = boost::vertices(graph_); v.first != v.second; ++v.first) {
      const VertexConstPtr vp = graph_[*v.first];
      if (isOptimized(*v.first)) {
        if (vp->isValue()) { numOptVal++;
        } else { numOptFac++; }
      } else {
        if (vp->isValue()) { numVal++;
        } else { numFac++; }
      }
    }
    std::stringstream ss;
    ss << "opt fac: " << numOptFac << " unopt fac: " << numFac
       << " opt vals: " << numOptVal << " unopt vals: " << numVal;
    return (ss.str());
  }

  void Graph::printUnoptimized() const {
    for (auto v = boost::vertices(graph_); v.first != v.second; ++v.first) {
      const VertexConstPtr vp = graph_[*v.first];
      if (!isOptimized(*v.first)) {
        ROS_INFO_STREAM("unoptimized: " << vp->getLabel());
      }
    }
  }

  double
  Graph::getError(const VertexDesc &v) const {
    const VertexConstPtr vp = graph_[v];
    VertexToOptMap::const_iterator it = optimized_.find(v);
    if (!vp->isValue() && it != optimized_.end()) {
      const FactorConstPtr fp =
        std::dynamic_pointer_cast<const factor::Factor>(vp);
      double errSum(0);
      for (const auto &k: it->second) {
        double e = optimizer_->getError(k);
        errSum += e;
      }
      return (errSum);
    }
    return (-1.0);
  }

  Graph::ErrorToVertexMap Graph::getErrorMap() const {
    ErrorToVertexMap errMap;
    for (auto vi = boost::vertices(graph_); vi.first != vi.second;
         ++vi.first) {
      const double err = getError(*vi.first);
      if (err >= 0) {
        errMap.insert(ErrorToVertexMap::value_type(err, *vi.first));
      }
    }
    return (errMap);
  }

  void Graph::printErrorMap(const string &prefix) const {
    for (auto vi = boost::vertices(graph_); vi.first != vi.second;
         ++vi.first) {
      const VertexDesc v = *vi.first;
      const VertexConstPtr vp = graph_[v];
      VertexToOptMap::const_iterator it = optimized_.find(v);
      if (!vp->isValue() && it != optimized_.end()) {
        double errSum(0);
        ROS_INFO_STREAM(prefix << " " << info(v) << ":");
        for (const auto &k: it->second) {
          double e = optimizer_->getError(k);
          errSum += e;
          optimizer_->printFactorError(k);
        }
      }
    }
  }

  Graph::TimeToErrorMap Graph::getTimeToErrorMap() const {
    TimeToErrorMap m;
    for (auto vi = boost::vertices(graph_); vi.first != vi.second;
         ++vi.first) {
      const VertexDesc v = *vi.first;
      const VertexConstPtr vp = graph_[v];
      VertexToOptMap::const_iterator it = optimized_.find(v);
      if (!vp->isValue() && it != optimized_.end()) {
        const FactorConstPtr fp =
          std::dynamic_pointer_cast<const factor::Factor>(vp);
        double errSum(0);
        for (const auto &k: it->second) {
          double e = optimizer_->getError(k);
          errSum += e;
        }
        auto it = m.find(fp->getTime());
        if (it == m.end()) {
          it = m.emplace(fp->getTime(),TimeToErrorMap::mapped_type()).first;
        }
        it->second.emplace_back(fp, errSum);
      }
    }
    return (m);
  }

  PoseNoise Graph::getPoseNoise(const VertexDesc &v) const {
    const ValueKey k = findOptimizedPoseKey(v);
    return (PoseNoise(optimizer_->getMarginal(k)));
  }
 
  // static method!
  string Graph::tag_name(int tagid) {
    return (string("tag:") + std::to_string(tagid));
  }
  // static method!
  string Graph::body_name(const string &body) {
    return ("body:" + body);
  }
  // static method!
  string Graph::cam_name(const string &cam) {
    return ("cam:" + cam);
  }

}  // end of namespace
