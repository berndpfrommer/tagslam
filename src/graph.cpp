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

  template <class G>
  class LabelWriter {
  public:
    LabelWriter(const G &g) : graph(&g) { }
    template <class VertexOrEdge>
    void operator()(std::ostream &out, const VertexOrEdge& v) const {
      VertexConstPtr vp = (*graph)[v];
      const std::string color =
        vp->isOptimized() ? "green" : (vp->isValid() ? "blue" : "red");
      out << "[label=\"" << vp->getLabel() << "\", shape="
          << vp->getShape() << ", color=" << color << "]";
    }
  private:
    const G *graph;
  };

  template<class G>
  static void plot(const std::string &fname, const G &graph) {
    std::ofstream ofile(fname);
    boost::write_graphviz(ofile, graph, LabelWriter<G>(graph));
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

    VertexDesc vcp = find(value::Pose::id(ros::Time(0),
                                      Graph::cam_name(pf->getCamera()->getName())));
    if (!is_valid(vcp)) {
      ROS_ERROR_STREAM("no camera pose found for: " << pf->getLabel());
      throw std::runtime_error("no camera pose found!");
    }
    VertexDesc v = insertVertex(pf);
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


  void Graph::addToOptimizer(const VertexDesc &v) {
    graph_[v]->addToOptimizer(this);
  }

  std::vector<ValueKey>
  Graph::getOptKeysForFactor(VertexDesc fv) const {
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
      if (!vp->isValid()) {
        ROS_ERROR_STREAM("vertex is not valid: " << vvp->getLabel());
        throw std::runtime_error("vertex is not valid");
      }
      if (!vp->isOptimized()) {
        ROS_DEBUG_STREAM("vertex is not optimized: " << vvp->getLabel());
        throw std::runtime_error("vertex is not optimized");
      }
      optKeys.push_back(vp->getKey());
    }
    return (optKeys);
  }

  OptimizerKey
  Graph::addToOptimizer(factor::AbsolutePosePrior *p) {
    if (p->isOptimized()) {
      ROS_ERROR_STREAM("already optimized: " << p->getLabel());
      throw std::runtime_error("already optimized!");
    }
    VertexDesc v = find(p->getId());
    if (!is_valid(v)) {
      ROS_ERROR_STREAM("cannot find factor " << p->getLabel());
      throw std::runtime_error("cannot find factor");
    }
    std::vector<ValueKey> optKeys = getOptKeysForFactor(v);
    if (optKeys.size() != 1) {
      ROS_ERROR_STREAM("wrong num values for " << info(v) << ": " << optKeys.size());
      throw std::runtime_error("wrong num values for factor");
    }
    p->setKey(optimizer_->addAbsolutePosePrior(optKeys[0], p->getPoseWithNoise()));
    return (p->getKey());
  }


  OptimizerKey
  Graph::addToOptimizer(factor::RelativePosePrior *p) {
    if (p->isOptimized()) {
      ROS_ERROR_STREAM("already optimized: " << p->getLabel());
      throw std::runtime_error("already optimized!");
    }
    VertexDesc v = find(p->getId());
    if (!is_valid(v)) {
      ROS_ERROR_STREAM("cannot find factor " << p->getLabel());
      throw std::runtime_error("cannot find factor");
    }
    std::vector<ValueKey> optKeys = getOptKeysForFactor(v);
    if (optKeys.size() != 2) {
      ROS_ERROR_STREAM("wrong num values for " << info(v) << ": " << optKeys.size());
      throw std::runtime_error("wrong num values for factor");
    }
    p->setKey(optimizer_->addRelativePosePrior(optKeys[0], optKeys[1], p->getPoseWithNoise()));
    return (p->getKey());
  }
  

  std::vector<OptimizerKey>
  Graph::addToOptimizer(factor::TagProjection *p) {
    if (p->isOptimized()) {
      ROS_ERROR_STREAM("already optimized: " << p->getLabel());
      throw std::runtime_error("already optimized!");
    }
    VertexDesc v = find(p->getId());
    if (!is_valid(v)) {
      ROS_ERROR_STREAM("cannot find factor " << p->getLabel());
      throw std::runtime_error("cannot find factor");
    }
    std::vector<ValueKey> optKeys = getOptKeysForFactor(v);
    if (optKeys.size() != 4) {
      ROS_ERROR_STREAM("wrong num values for " << info(v) << ": " << optKeys.size());
      throw std::runtime_error("wrong num values for factor");
    }
    p->setKeys(
      optimizer_->addTagProjectionFactor(
        p->getImageCorners(), p->getTag()->getObjectCorners(),
        p->getCamera()->getName(), p->getCamera()->getIntrinsics(),
        p->getPixelNoise(), optKeys[0], optKeys[1], optKeys[2], optKeys[3]));
    return (p->getKeys());
  }

  VertexDesc
  Graph::addPose(const ros::Time &t, const string &name,
                 const Transform &pose, bool poseIsValid, bool isCameraPose) {
    if (hasId(value::Pose::id(t, name))) {
      ROS_ERROR_STREAM("duplicate pose inserted: " << t << " " << name);
      throw (std::runtime_error("duplicate pose inserted"));
    }
    PoseValuePtr pv(new value::Pose(t, pose, name, poseIsValid, isCameraPose));
    return (insertVertex(pv));
  }

  OptimizerKey
  Graph::addToOptimizer(value::Pose *p) {
    if (p->isOptimized()) {
      ROS_ERROR_STREAM("already optimized: " << p->getLabel());
      throw std::runtime_error("already optimized!");
    }
    p->setKey(optimizer_->addPose(p->getPose()));
    return (p->getKey());
  }

  Transform Graph::getOptimizedPose(const VertexDesc &v) const {
    PoseValueConstPtr  vp = std::dynamic_pointer_cast<value::Pose>(graph_[v]);
    if (!vp) {
      ROS_ERROR_STREAM("vertex is not pose: " << info(v));
      throw std::runtime_error("vertex is not pose");
    }
    if (!vp->isOptimized()) {
      ROS_ERROR_STREAM("get opt pose: vertex not optimized: " << info(v));
      throw std::runtime_error("vertex is not optimized");
    }
    return (optimizer_->getPose(vp->getKey()));
  }

  void
  Graph::copyFrom(const Graph &g, const std::deque<VertexDesc> &srcfacs,
                  std::deque<VertexDesc> *destfacs) {
    ROS_DEBUG_STREAM("copying from...");
    std::set<VertexDesc> copiedVals;
    // first copy all values
    for (const auto &srcf: srcfacs) { // loop through factors
      ROS_DEBUG_STREAM(" copying for factor " << g.info(srcf));
      for (const auto &srcv: g.getConnected(srcf)) {
        if (copiedVals.count(srcv) == 0) { // avoid duplication
          ROS_DEBUG_STREAM("  copying value " << *g.getVertex(srcv));
          copiedVals.insert(srcv);
          PoseValuePtr vp =
            std::dynamic_pointer_cast<value::Pose>(g.getVertex(srcv));
          if (vp) {
            if (vp->isOptimized()) {
              vp->setPose(g.getOptimizedPose(srcv));
            }
            VertexDesc destv = vp->attachTo(this);  // makes copy of vp
            if (vp->isOptimized()) {
              // known pose, so can already add it to optimizer
              getVertex(destv)->addToOptimizer(this);
              // already established poses must be pinned down with a prior
              double ns = vp->isCameraPose() ? 0.1 : 0.001;
              AbsolutePosePriorFactorPtr
                pp(new factor::AbsolutePosePrior(
                     vp->getTime(),
                     PoseWithNoise(vp->getPose(), PoseNoise2::make(ns, ns), true), vp->getName()));
              add(pp);
              addToOptimizer(pp.get());
              //ROS_DEBUG_STREAM("adding prior to free pose: " << *pp);
              //std::cout << pp->getPoseWithNoise().getPose() << std::endl;
            }
            //ROS_DEBUG_STREAM("attached: " << *graph_[destv]);
            //graph_[destv]->addToOptimizer(this);
          } else {
            ROS_ERROR_STREAM("unexpected type: " << g.getVertex(srcv)->getLabel());
            throw (std::runtime_error("expected value, got fac"));
          }
        }
      }
    }
    // now copy factors
    for (const auto &srcf: srcfacs) { // loop through factors
      FactorPtr fp = std::dynamic_pointer_cast<factor::Factor>(g.getVertex(srcf));
      if (fp) {
        destfacs->push_back(fp->attachTo(this));
      }
    }
  }

  void Graph::plotDebug(const ros::Time &t, const string &tag) {
    std::stringstream ss;
    ss << tag << "_" <<  t.toNSec() << ".dot";
    plot(ss.str(), graph_);
  }


  void Graph::transferOptimizedPose(const VertexDesc &v) {
    VertexPtr vp = getVertex(v);
    if (vp->isValue() && vp->isOptimized()) {
      PoseValuePtr p = std::dynamic_pointer_cast<value::Pose>(vp);
      if (p) {
        p->setPose(optimizer_->getPose(p->getKey()));
      }
    }
  }

  void Graph::transferOptimizedValues() {
    for (auto vi = boost::vertices(graph_); vi.first != vi.second; ++vi.first) {
      transferOptimizedPose(*vi.first);
    }
  }

  std::string Graph::info(const VertexDesc &v) const {
    return (graph_[v]->getLabel());
  }

  void Graph::initializeFrom(const Graph &sg) {
    // first initialize all values and add to optimizer
    int numTransferredPoses(0);
    for (auto vi = boost::vertices(sg.graph_); vi.first != vi.second; ++vi.first) {
      PoseValuePtr   psp = std::dynamic_pointer_cast<value::Pose>(sg.graph_[*vi.first]);
      if (psp) {
        VertexDesc dv = find(psp->getId());
        if (!is_valid(dv)) {
          ROS_ERROR_STREAM("invalid init value: " << psp->getLabel());
          throw std::runtime_error("invalid init value");
        }
        PoseValuePtr pdp = std::dynamic_pointer_cast<value::Pose>(graph_[dv]);
        if (!pdp) {
          ROS_ERROR_STREAM("invalid init type: " << graph_[dv]->getLabel());
          throw std::runtime_error("invalid init type");
        }
        if (!pdp->isOptimized()) {
          ROS_DEBUG_STREAM("transferring pose from graph: " << pdp->getLabel());
          pdp->setPose(psp->getPose());
          pdp->addToOptimizer(this);
          numTransferredPoses++;
        }
      }
    }
    // now add all necessary factors to optimizer
    for (auto vi = boost::vertices(sg.graph_); vi.first != vi.second; ++vi.first) {
      VertexPtr sp = sg.graph_[*vi.first];
      if (std::dynamic_pointer_cast<factor::TagProjection>(sp) ||
          std::dynamic_pointer_cast<factor::RelativePosePrior>(sp)) {
        ROS_DEBUG_STREAM("transferring factor: " << *sp);
        VertexDesc dv = find(sp->getId());
        if (is_valid(dv)) {
          getVertex(dv)->addToOptimizer(this);
#if 0          
          if (numTransferredPoses <= 1 && fooCnt_ > 2100) {
            double error = optimizer_->errorFull();
            ROS_DEBUG_STREAM(fooCnt_ << " error after factor: " << error);
          }
#endif          
        } else {
          ROS_ERROR_STREAM("no orig vertex found for: " << *sp);
          throw std::runtime_error("no orig vertex found");
        }
      }
    }
  }
#if 0
  // static method!
  void Graph::transfer_optimized_pose(
    const std::shared_ptr<Graph> &destGraph, const VertexDesc &destVertex,
    const Graph &srcGraph, const VertexDesc &srcVertex) {
    VertexPtr vp = srcGraph.getVertex(srcVertex);
    if (vp->isValue() && vp->isOptimized()) {
      PoseValuePtr psrc = std::dynamic_pointer_cast<value::Pose>(vp);
      if (psrc) {
        PoseValuePtr pdest = std::dynamic_pointer_cast<value::Pose>(destGraph->getVertex(destVertex));
        if (pdest) {
          pdest->setPose(srcGraph.optimizer_->getPose(psrc->getKey()));
        }
      }
    }
  }
#endif  

  bool Graph::hasPose(const ros::Time &t,
                      const std::string &name) const {
    return (hasId(value::Pose::id(t, name)));
  }

  VertexDesc Graph::findPose(const ros::Time &t, const string &name) const {
    return (find(value::Pose::id(t, name)));
  }

  void Graph::setOptimizedPose(const VertexDesc v,
                               const Transform &pose) {
    PoseValueConstPtr psrc = std::dynamic_pointer_cast<const value::Pose>(graph_[v]);
    optimizer_->setPose(psrc->getKey(), pose);
  }

  void Graph::print(const std::string &prefix) const {
    for (auto vi = boost::vertices(graph_); vi.first != vi.second; ++vi.first) {
      ROS_DEBUG_STREAM(prefix << " " << graph_[*vi.first]->getLabel());
      if (graph_[*vi.first]->isValue()) {
        PoseValueConstPtr  vp = std::dynamic_pointer_cast<const value::Pose>(graph_[*vi.first]);
        if (vp) {
          std::cout << " has pose: " << std::endl << vp->getPose() << std::endl;
        }
      }
    }
  }

  std::string Graph::getStats() const {
    int numFac(0), numOptFac(0), numVal(0), numOptVal(0);
    for (auto vi = boost::vertices(graph_); vi.first != vi.second; ++vi.first) {
      const VertexConstPtr vp = graph_[*vi.first];
      if (vp->isOptimized()) {
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

  Graph::ErrorToVertexMap Graph::getErrorMap() const {
    ErrorToVertexMap errMap;
    for (auto vi = boost::vertices(graph_); vi.first != vi.second; ++vi.first) {
      const VertexConstPtr vp = graph_[*vi.first];
      if (!vp->isValue() && vp->isOptimized()) {
        const FactorConstPtr fp = std::dynamic_pointer_cast<const factor::Factor>(vp);
        double errSum(0);
        for (const auto &k: fp->getKeys()) {
          double e = optimizer_->getError(k);
          errSum += e;
        }
        errMap.insert(ErrorToVertexMap::value_type(errSum, *vi.first));
      }
    }
    return (errMap);
  }


  PoseNoise2 Graph::getPoseNoise(const VertexDesc &v) const {
    PoseValueConstPtr  vp = std::dynamic_pointer_cast<const value::Pose>(graph_[v]);
    if (!vp) {
      ROS_ERROR_STREAM("cannot get pose noise for invalid vertex: " << v);
      throw std::runtime_error("cannot get pose noise for invalid vertex");
    }
    return (PoseNoise2(optimizer_->getMarginal(vp->getKey())));
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
