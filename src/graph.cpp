/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph.h"
#include "tagslam/body.h"
#include "tagslam/camera2.h"
#include "tagslam/gtsam_optimizer.h"
#include "tagslam/value/pose.h"
#include "tagslam/factor/absolute_pose_prior.h"
#include "tagslam/factor/relative_pose_prior.h"
#include "tagslam/factor/tag_projection.h"
#include <boost/range/irange.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_utility.hpp>


#include <sstream>

namespace tagslam {

  using boost::irange;

  class LabelWriter {
  public:
    LabelWriter(const Graph *g) : graph_(g)  { }
    template <class VertexOrEdge>
    void operator()(std::ostream &out, const VertexOrEdge& v) const {
      VertexConstPtr vp = graph_->getVertex(v);
      const std::string color =  graph_->isOptimized(v) ? "green" : "red";
      out << "[label=\"" << vp->getLabel() << "\", shape="
          << vp->getShape() << ", color=" << color << "]";
    }
  private:
    const Graph *graph_;
  };

  static void plot(const std::string &fname, const Graph *g) {
    std::ofstream ofile(fname);
    boost::write_graphviz(ofile, g->getBoostGraph(), LabelWriter(g));
  }

  Graph::Graph() {
    optimizer_.reset(new GTSAMOptimizer());
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

  VertexDesc Graph::add(const PoseValuePtr &p) {
    return (insertVertex(p));
  }

  VertexDesc
  Graph::add(const AbsolutePosePriorFactorPtr &fac) {
    VertexDesc cp = find(value::Pose::id(fac->getTime(), fac->getName()));
    if (!is_valid(cp)) {
      ROS_ERROR_STREAM("pose missing: " << fac->getName() << " " << cp);
      throw std::runtime_error("pose missing!");
    }
    VertexDesc fv = insertVertex(fac);
    factors_.push_back(fv);
    boost::add_edge(fv, cp, GraphEdge(0), graph_);
    return (fv);
  }

  VertexDesc
  Graph::add(const RelativePosePriorFactorPtr &p) {
    VertexDesc pp = findPose(p->getPreviousTime(), p->getName());
    VertexDesc cp = findPose(p->getTime(), p->getName());
    if (!is_valid(pp) || !is_valid(cp)) {
      ROS_ERROR_STREAM("pose missing: " << p->getLabel() << " " << pp << " " << cp);
      throw std::runtime_error("pose missing!");
    }
    VertexDesc fv = insertVertex(p);
    factors_.push_back(fv);
    boost::add_edge(fv, pp, GraphEdge(0), graph_);
    boost::add_edge(fv, cp, GraphEdge(1), graph_);
    return (fv);
  }

  VertexDesc
  Graph::add(const TagProjectionFactorPtr &pf) {
    // connect: tag_body_pose, tag_pose, cam_pose, rig_pose
    VertexDesc vtp = find(value::Pose::id(ros::Time(0),
                                      Graph::tag_name(pf->getTag()->getId())));
    if (!is_valid(vtp)) {
      ROS_ERROR_STREAM("no tag pose found for: " << pf->getLabel());
      throw std::runtime_error("no tag pose found!");
    }
    BodyConstPtr body = pf->getTag()->getBody();
    VertexDesc vbp = find(value::Pose::id(body->isStatic() ? ros::Time(0) : pf->getTime(),
                                      Graph::body_name(body->getName())));
    if (!is_valid(vbp)) {
      ROS_ERROR_STREAM("no body pose found for: " << pf->getLabel());
      throw std::runtime_error("no body pose found!");
    }
    BodyConstPtr rig = pf->getCamera()->getRig();
    VertexDesc vrp = find(value::Pose::id(rig->isStatic() ? ros::Time(0) : pf->getTime(),
                                      Graph::body_name(rig->getName())));
    if (!is_valid(vrp)) {
      ROS_ERROR_STREAM("no rig pose found for: " << pf->getLabel());
      throw std::runtime_error("no rig pose found!");
    }

    VertexDesc vcp = find(value::Pose::id(pf->getTime(),
                                      Graph::cam_name(pf->getCamera()->getName())));
    if (!is_valid(vcp)) {
      ROS_ERROR_STREAM("no camera pose found for: " << pf->getLabel());
      throw std::runtime_error("no camera pose found!");
    }
    VertexDesc v = insertVertex(pf);
    factors_.push_back(v);
    boost::add_edge(v, vcp, GraphEdge(0), graph_); // T_r_c
    boost::add_edge(v, vrp, GraphEdge(1), graph_); // T_w_r
    boost::add_edge(v, vbp, GraphEdge(2), graph_); // T_w_b
    boost::add_edge(v, vtp, GraphEdge(3), graph_); // T_b_o
    return (v);
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


  std::vector<ValueKey>
  Graph::getOptKeysForFactor(VertexDesc fv, int numKeys) const {
    auto edges = boost::out_edges(fv, graph_);
    std::vector<ValueKey> optKeys;
    for (auto edgeIt = edges.first; edgeIt != edges.second; ++edgeIt) {
      VertexDesc vv  = boost::target(*edgeIt, graph_); // value vertex
      VertexPtr     vvp = graph_[vv]; // pointer to value
      ValuePtr       vp = std::dynamic_pointer_cast<value::Value>(vvp);
      if (!vp) {
        ROS_ERROR_STREAM("vertex is no pose: " << vvp->getLabel());
        throw std::runtime_error("vertex is no pose");
      }
      optKeys.push_back(findOptimizedPoseKey(vv));
    }
    if (optKeys.size() != (size_t)numKeys) {
      ROS_ERROR_STREAM("wrong num values for " << info(fv) << ": "
                       << optKeys.size() << " expected: " << numKeys);
      throw std::runtime_error("wrong num values for factor");
    }

    return (optKeys);
  }


  VertexDesc Graph::find(const Vertex *vp) const {
    VertexDesc v = find(vp->getId());
    if (!is_valid(v)) {
      ROS_ERROR_STREAM("cannot find factor " << vp->getLabel());
      throw std::runtime_error("cannot find " + vp->getLabel());
    }
    return (v);
  }

  Graph::VertexToOptMap::const_iterator
  Graph::findOptimized(const VertexDesc &v) const {
    VertexToOptMap::const_iterator it = optimized_.find(v);
    if (it == optimized_.end()) {
      ROS_ERROR_STREAM("not optimized: " << info(v));
      throw std::runtime_error("not optimized: " + info(v));
    }
    return (it);
  }

  void Graph::verifyUnoptimized(const VertexDesc &v) const {
    const VertexToOptMap::const_iterator it = optimized_.find(v);
    if (it != optimized_.end()) {
      ROS_ERROR_STREAM("already optimized: " << info(v));
      throw std::runtime_error("already optimized: " + info(v));
    }
  }

  ValueKey Graph::findOptimizedPoseKey(const VertexDesc &v) const {
    VertexToOptMap::const_iterator it = optimized_.find(v);
    if (it == optimized_.end()) {
      ROS_ERROR_STREAM("cannot find opt pose: " << info(v));
      throw std::runtime_error("cannot find opt pose: " + info(v));
    }
    if (it->second.size() != 1) {
      ROS_DEBUG_STREAM("pose must have one opt value: " << info(v)
                       << " but has: " << it->second.size());
      throw std::runtime_error("pose must have one opt value!");
    }
    return (it->second[0]);
  }
  
  OptimizerKey
  Graph::addToOptimizer(const factor::AbsolutePosePrior *p) {
    VertexDesc v = find(p);
    verifyUnoptimized(v);
    std::vector<ValueKey> optKeys = getOptKeysForFactor(v, 1);
    FactorKey fk =
      optimizer_->addAbsolutePosePrior(optKeys[0], p->getPoseWithNoise());
    optimized_.insert(
      VertexToOptMap::value_type(v, std::vector<FactorKey>(1, fk)));
    return (fk);
  }


  OptimizerKey
  Graph::addToOptimizer(const factor::RelativePosePrior *p) {
    VertexDesc v = find(p);
    verifyUnoptimized(v);
    std::vector<ValueKey> optKeys = getOptKeysForFactor(v, 2);
    FactorKey fk = optimizer_->addRelativePosePrior(optKeys[0], optKeys[1],
                                                    p->getPoseWithNoise());
    optimized_.insert(
      VertexToOptMap::value_type(v, std::vector<FactorKey>(1, fk)));
    return (fk);
  }

  std::vector<OptimizerKey>
  Graph::addToOptimizer(const factor::TagProjection *p) {
    VertexDesc v = find(p);
    verifyUnoptimized(v);
    std::vector<ValueKey> optKeys = getOptKeysForFactor(v, 4);
    std::vector<FactorKey> fks = 
      optimizer_->addTagProjectionFactor(
        p->getImageCorners(), p->getTag()->getObjectCorners(),
        p->getCamera()->getName(), p->getCamera()->getIntrinsics(),
        p->getPixelNoise(), optKeys[0], optKeys[1], optKeys[2], optKeys[3]);
    optimized_.insert(VertexToOptMap::value_type(v, fks));
    return (fks);
  }

  OptimizerKey
  Graph::addToOptimizer(const VertexDesc &v, const Transform &tf) {
    ROS_DEBUG_STREAM("adding pose to opt: " << info(v));
    verifyUnoptimized(v);
    ValueKey vk = optimizer_->addPose(tf);
    auto fk = std::vector<FactorKey>(1, vk);
    optimized_.insert(VertexToOptMap::value_type(v, fk));
    return (vk);
  }

  VertexDesc
  Graph::addPose(const ros::Time &t, const string &name,
                 bool isCameraPose) {
    if (hasId(value::Pose::id(t, name))) {
      ROS_ERROR_STREAM("duplicate pose inserted: " << t << " " << name);
      throw (std::runtime_error("duplicate pose inserted"));
    }
    PoseValuePtr pv(new value::Pose(t, name, isCameraPose));
    return (insertVertex(pv));
  }


  Transform Graph::getOptimizedPose(const VertexDesc &v) const {
    PoseValueConstPtr vp = std::dynamic_pointer_cast<value::Pose>(graph_[v]);
    if (!vp) {
      ROS_ERROR_STREAM("vertex is not pose: " << info(v));
      throw std::runtime_error("vertex is not pose");
    }
    VertexToOptMap::const_iterator it = findOptimized(v);
    return (optimizer_->getPose(it->second[0]));
  }

  
  void
  Graph::copyFrom(const Graph &g, const std::deque<VertexDesc> &srcfacs,
                  std::deque<VertexDesc> *destfacs) {
    std::set<VertexDesc> copiedVals;
    // first copy all values
    for (const auto &srcf: srcfacs) { // loop through factors
      //ROS_DEBUG_STREAM(" copying for factor " << g.info(srcf));
      for (const auto &srcv: g.getConnected(srcf)) {
        if (copiedVals.count(srcv) == 0) { // avoid duplication
          //ROS_DEBUG_STREAM("  copying value " << *g.getVertex(srcv));
          copiedVals.insert(srcv);
          GraphVertex srcvp  = g.getVertex(srcv);
          GraphVertex destvp = srcvp->clone();
          destvp->attach(destvp, this); // add new value to graph
          if (g.isOptimized(srcv)) {
            // Already established poses must be pinned down with a prior
            // If it's a camera pose, give it more flexibility
            PoseValuePtr srcpp =
              std::dynamic_pointer_cast<value::Pose>(srcvp);
            Transform pose = g.getOptimizedPose(srcv);
            double ns = srcpp->isCameraPose() ? 0.1 : 0.001;
            PoseWithNoise pwn(pose, PoseNoise2::make(ns, ns), true);
            AbsolutePosePriorFactorPtr
              pp(new factor::AbsolutePosePrior(destvp->getTime(), pwn,
                                               destvp->getName()));
            // Add pose prior to graph
            VertexDesc destppv = add(pp);
            destfacs->push_back(destppv);
          }
        }
      }
    }
    // now copy factors
    for (const auto &srcf: srcfacs) { // loop through factors
      FactorPtr fp =
        std::dynamic_pointer_cast<factor::Factor>(g.getVertex(srcf));
      if (fp) {
        GraphVertex destfp = fp->clone();
        destfacs->push_back(destfp->attach(destfp, this));
      }
    }
  }

  void Graph::plotDebug(const ros::Time &t, const string &tag) {
    std::stringstream ss;
    ss << tag << "_" <<  t.toNSec() << ".dot";
    plot(ss.str(), this);
  }

  std::string Graph::info(const VertexDesc &v) const {
    return (graph_[v]->getLabel());
  }

  void Graph::initializeFrom(const Graph &sg) {
    // first initialize all values and add to optimizer
    int numTransferredPoses(0);
    for (auto vi = boost::vertices(sg.graph_); vi.first != vi.second;
         ++vi.first) {
      const VertexDesc sv = *vi.first;
      PoseValuePtr psp =
        std::dynamic_pointer_cast<value::Pose>(sg.graph_[sv]);
      if (psp) {
        const VertexDesc dv = find(psp->getId());
        if (!is_valid(dv)) {
          ROS_ERROR_STREAM("cannot find dest value: " << psp->getLabel());
          throw std::runtime_error("cannot find dest value");
        }
        PoseValuePtr pdp = std::dynamic_pointer_cast<value::Pose>(graph_[dv]);
        if (!pdp) {
          ROS_ERROR_STREAM("invalid dest type: " << graph_[dv]->getLabel());
          throw std::runtime_error("invalid dest type");
        }
        if (!isOptimized(dv) && sg.isOptimized(sv)) {
          //ROS_DEBUG_STREAM("transferring pose: " << pdp->getLabel());
          addToOptimizer(dv, sg.getOptimizedPose(sv));
          numTransferredPoses++;
        }
      }
    }
    // now add all necessary factors to optimizer
    for (auto vi = boost::vertices(sg.graph_); vi.first != vi.second;
         ++vi.first) {
      const VertexDesc sv = *vi.first;
      const FactorConstPtr sfp =
        std::dynamic_pointer_cast<factor::Factor>(sg.graph_[sv]);
      if (sfp && !std::dynamic_pointer_cast<
          factor::AbsolutePosePrior>(sg.graph_[sv])) {
        //ROS_DEBUG_STREAM("transferring factor: " << sg.info(sv));
        VertexDesc dv = find(sfp->getId());
        if (is_valid(dv)) {
          sfp->addToOptimizer(this);
        } else {
          ROS_ERROR_STREAM("no orig vertex found for: " << sg.info(sv));
          throw std::runtime_error("no orig vertex found");
        }
      }
    }
  }

  bool Graph::hasPose(const ros::Time &t,
                      const std::string &name) const {
    return (hasId(value::Pose::id(t, name)));
  }

  VertexDesc Graph::findPose(const ros::Time &t, const string &name) const {
    return (find(value::Pose::id(t, name)));
  }

  void Graph::print(const std::string &prefix) const {
    for (auto vi = boost::vertices(graph_); vi.first != vi.second; ++vi.first) {
      bool isOpt = (optimized_.find(*vi.first) != optimized_.end());
      ROS_DEBUG_STREAM(prefix << " " << graph_[*vi.first]->getLabel()
                       << ":" << (isOpt ? "O":"U"));
      if (graph_[*vi.first]->isValue()) {
        PoseValueConstPtr  vp = std::dynamic_pointer_cast<const value::Pose>(graph_[*vi.first]);
      }
    }
  }

  std::string Graph::getStats() const {
    int numFac(0), numOptFac(0), numVal(0), numOptVal(0);
    for (auto vi = boost::vertices(graph_); vi.first != vi.second; ++vi.first) {
      const VertexConstPtr vp = graph_[*vi.first];
      if (isOptimized(*vi.first)) {
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
    for (auto vi = boost::vertices(graph_); vi.first != vi.second; ++vi.first) {
      const VertexConstPtr vp = graph_[*vi.first];
      if (!isOptimized(*vi.first)) {
        ROS_INFO_STREAM("unoptimized: " << vp->getLabel());
      }
    }
  }

  Graph::ErrorToVertexMap Graph::getErrorMap() const {
    ErrorToVertexMap errMap;
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
        errMap.insert(ErrorToVertexMap::value_type(errSum, v));
      }
    }
    return (errMap);
  }


  PoseNoise2 Graph::getPoseNoise(const VertexDesc &v) const {
    const ValueKey k = findOptimizedPoseKey(v);
    return (PoseNoise2(optimizer_->getMarginal(k)));
  }
 
  // static method!
  std::string Graph::tag_name(int tagid) {
    return (string("tag:") + std::to_string(tagid));
  }
  // static method!
  std::string Graph::body_name(const string &body) {
    return ("body:" + body);
  }
  // static method!
  std::string Graph::cam_name(const string &cam) {
    return ("cam:" + cam);
  }

}  // end of namespace
