/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph.h"
#include "tagslam/body.h"
#include "tagslam/gtsam_optimizer.h"
#include "tagslam/value/pose.h"
#include "tagslam/factor/absolute_pose_prior.h"
#include "tagslam/factor/relative_pose_prior.h"
#include "tagslam/factor/tag_projection.h"
#include <boost/range/irange.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_utility.hpp>

namespace tagslam {

  using boost::irange;

  template <class G>
  class LabelWriter {
  public:
    LabelWriter(const G &g) : graph(&g) { }
    template <class VertexOrEdge>
    void operator()(std::ostream &out, const VertexOrEdge& v) const {
      VertexConstPtr vp = (*graph)[v].vertex;
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

  void Graph::optimize() {
    if (optimizer_) {
      optimizer_->optimize();
    }
  }
  
  void Graph::optimizeFull(bool force) {
    if (optimizer_) {
      optimizer_->optimizeFull(force);
    }
  }

  std::vector<Graph::Vertex> Graph::getConnected(const Vertex &v) const {
    auto edges = boost::out_edges(v, graph_);
    std::vector<Vertex> c;
    for (auto edgeIt = edges.first; edgeIt != edges.second; ++edgeIt) {
      c.push_back(boost::target(*edgeIt, graph_));
    }
    return (c);
  }

  Graph::Vertex
  Graph::addVertex(const VertexPtr &vp, const std::string &id) {
    const Graph::Vertex nv = boost::add_vertex(GraphVertex(vp), graph_);
    idToVertex_.insert(IdToVertexMap::value_type(id, nv));
    return (nv);
  }

  std::vector<ValueKey>
  Graph::getOptKeysForFactor(Graph::Vertex fv) const {
    auto edges = boost::out_edges(fv, graph_);
    std::vector<ValueKey> optKeys;
    for (auto edgeIt = edges.first; edgeIt != edges.second; ++edgeIt) {
      Graph::Vertex vv  = boost::target(*edgeIt, graph_); // value vertex
      VertexPtr     vvp = graph_[vv].vertex; // pointer to value
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

  Graph::Vertex
  Graph::addAbsolutePosePriorFactor(const ros::Time &t, const string &name,
                                    const PoseWithNoise &pn) {
    std::shared_ptr<factor::AbsolutePosePrior>
      fac(new factor::AbsolutePosePrior(t, pn, name));
    const std::string id = make_id(t, name);
    Vertex cp = find(id);
    if (!is_valid(cp)) {
      ROS_ERROR_STREAM("pose missing: " << name << " " << cp);
      throw std::runtime_error("pose missing!");
    }
    Graph::Vertex fv = addVertex(fac, Graph::make_id(t, "app_" + name));
    boost::add_edge(fv, cp, GraphEdge(0), graph_);
    return (fv);
  }

  void
  Graph::addAbsolutePosePriorFactorToOptimizer(const Graph::Vertex &v) {
    auto &vp = graph_[v].vertex;
    if (vp->isOptimized()) {
      ROS_ERROR_STREAM("already optimized: " << vp->getLabel());
      throw std::runtime_error("already optimized!");
    }
    AbsolutePosePriorFactorPtr p = std::dynamic_pointer_cast<factor::AbsolutePosePrior>(vp);
    if (!p) {
      ROS_ERROR_STREAM("factor is not abs pose prior: " << info(v));
      throw std::runtime_error("factor is not abs pose prior");
    }
    std::vector<ValueKey> optKeys = getOptKeysForFactor(v);
    if (optKeys.size() != 1) {
      ROS_ERROR_STREAM("wrong num values for " << info(v) << ": " << optKeys.size());
      throw std::runtime_error("wrong num values for factor");
    }
    p->setKey(optimizer_->addAbsolutePosePrior(optKeys[0], p->getPoseWithNoise()));
  }

  Graph::Vertex
  Graph::addRelativePosePriorFactor(const ros::Time &tPrev, const ros::Time &tCurr,
                                    const std::string &name,
                                    const PoseWithNoise &deltaPose) {
    FactorPtr f(new factor::RelativePosePrior(tCurr, tPrev, deltaPose, name));

    Vertex pp = find(Graph::make_id(tPrev, name));
    Vertex cp = find(Graph::make_id(tCurr, name));
    if (!is_valid(pp) || !is_valid(cp)) {
      ROS_ERROR_STREAM("pose missing: " << f->getLabel() << " " << pp << " " << cp);
      throw std::runtime_error("pose missing!");
    }
    int fv = addVertex(f, Graph::make_id(tCurr, "rpp_" + name));
    boost::add_edge(fv, pp, GraphEdge(0), graph_);
    boost::add_edge(fv, cp, GraphEdge(1), graph_);
    return (fv);
  }

  void
  Graph::addRelativePosePriorFactorToOptimizer(const Graph::Vertex &v) {
    auto &vp = graph_[v].vertex;
    if (vp->isOptimized()) {
      ROS_ERROR_STREAM("already optimized: " << vp->getLabel());
      throw std::runtime_error("already optimized!");
    }
    RelativePosePriorFactorPtr p = std::dynamic_pointer_cast<factor::RelativePosePrior>(vp);
    if (!p) {
      ROS_ERROR_STREAM("factor is not relative pose prior: " << info(v));
      throw std::runtime_error("factor is not rel pose prior");
    }
    std::vector<ValueKey> optKeys = getOptKeysForFactor(v);
    if (optKeys.size() != 2) {
      ROS_ERROR_STREAM("wrong num values for " << info(v) << ": " << optKeys.size());
      throw std::runtime_error("wrong num values for factor");
    }
    p->setKey(optimizer_->addRelativePosePrior(optKeys[0], optKeys[1], p->getPoseWithNoise()));
  }
  

  Graph::Vertex
  Graph::addTagProjectionFactor(const ros::Time &t,
                                const Tag2ConstPtr &tag,
                                const Camera2ConstPtr &cam,
                                double noise,
                                const geometry_msgs::Point *imgCorners) {
    TagProjectionFactorPtr pf(
      new factor::TagProjection(t, cam, tag, imgCorners, noise,
                                cam->getName() + "-" + tag_name(tag->getId())));

    // connect: tag_body_pose, tag_pose, cam_pose, rig_pose
    Vertex vtp = find(make_id(ros::Time(0),
                              Graph::tag_name(pf->getTag()->getId())));
    if (!is_valid(vtp)) {
      ROS_ERROR_STREAM("no tag pose found for: " << pf->getLabel());
      throw std::runtime_error("no tag pose found!");
    }
    BodyConstPtr body = tag->getBody();
    Vertex vbp = find(make_id(body->isStatic() ? ros::Time(0) : t,
                              Graph::body_name(body->getName())));
    if (!is_valid(vbp)) {
      ROS_ERROR_STREAM("no body pose found for: " << pf->getLabel());
      throw std::runtime_error("no body pose found!");
    }
    BodyConstPtr rig = cam->getRig();
    Vertex vrp = find(make_id(rig->isStatic() ? ros::Time(0) : t,
                              Graph::body_name(rig->getName())));
    if (!is_valid(vrp)) {
      ROS_ERROR_STREAM("no rig pose found for: " << pf->getLabel());
      throw std::runtime_error("no rig pose found!");
    }

    Vertex vcp = find(make_id(ros::Time(0),
                              Graph::cam_name(cam->getName())));
    if (!is_valid(vcp)) {
      ROS_ERROR_STREAM("no camera pose found for: " << pf->getLabel());
      throw std::runtime_error("no camera pose found!");
    }
    Graph::Vertex v =
      addVertex(pf, Graph::make_id(pf->getTime(), "tpf_" + pf->getName()));
    boost::add_edge(v, vcp, GraphEdge(0), graph_); // T_r_c
    boost::add_edge(v, vrp, GraphEdge(1), graph_); // T_w_r
    boost::add_edge(v, vbp, GraphEdge(2), graph_); // T_w_b
    boost::add_edge(v, vtp, GraphEdge(3), graph_); // T_b_o
    return (v);
  }

  void
  Graph::addTagProjectionFactorToOptimizer(const Graph::Vertex &v) {
    auto &vp = graph_[v].vertex;
    if (vp->isOptimized()) {
      ROS_ERROR_STREAM("already optimized: " << vp->getLabel());
      throw std::runtime_error("already optimized!");
    }
    TagProjectionFactorPtr p = std::dynamic_pointer_cast<factor::TagProjection>(vp);
    if (!p) {
      ROS_ERROR_STREAM("factor is not tag proj: " << info(v));
      throw std::runtime_error("factor is not tag proj");
    }
    std::vector<ValueKey> optKeys = getOptKeysForFactor(v);
    if (optKeys.size() != 4) {
      ROS_ERROR_STREAM("wrong num values for " << info(v) << ": " << optKeys.size());
      throw std::runtime_error("wrong num values for factor");
    }
    p->setKey(
      optimizer_->addTagProjectionFactor(
        p->getImageCorners(), p->getTag()->getObjectCorners(),
        p->getCamera()->getName(), p->getCamera()->getIntrinsics(),
        p->getPixelNoise(), optKeys[0], optKeys[1], optKeys[2], optKeys[3]));
  }

  Graph::Vertex
  Graph::addPose(const ros::Time &t, const string &name,
                 const Transform &pose, bool poseIsValid) {
    if (hasId(make_id(t, name))) {
      ROS_ERROR_STREAM("duplicate pose inserted: " << t << " " << name);
      throw (std::runtime_error("duplicate pose inserted"));
    }
    PoseValuePtr pv(new value::Pose(t, pose, name, poseIsValid));
    return (addVertex(pv, make_id(t, name)));
  }

  void
  Graph::addPoseToOptimizer(const Graph::Vertex &v) {
    auto &vp = graph_[v].vertex;
    if (vp->isOptimized()) {
      ROS_ERROR_STREAM("already optimized: " << vp->getLabel());
      throw std::runtime_error("already optimized!");
    }
    PoseValuePtr p = std::dynamic_pointer_cast<value::Pose>(vp);
    if (!p) {
      ROS_ERROR_STREAM("invalid type for value " << vp->getLabel());
      throw std::runtime_error("invalid type for value");
    }
    p->setKey(optimizer_->addPose(p->getPose()));
  }

  Transform Graph::getOptimizedPose(const Vertex &v) const {
    PoseValueConstPtr  vp = std::dynamic_pointer_cast<value::Pose>(graph_[v].vertex);
    if (!vp) {
      ROS_ERROR_STREAM("vertex is not pose: " << info(v));
      throw std::runtime_error("vertex is not pose");
    }
    if (!vp->isOptimized()) {
      ROS_ERROR_STREAM("vertex is not optimized: " << info(v));
      throw std::runtime_error("vertex is not optimized");
    }
    return (optimizer_->getPose(vp->getKey()));
  }

  void Graph::plotDebug(const ros::Time &t, const string &tag) {
    std::stringstream ss;
    ss << tag << "_" <<  t.toNSec() << ".dot";
    plot(ss.str(), graph_);
  }

  void Graph::transferOptimizedValues() {
    for (auto vi = boost::vertices(graph_); vi.first != vi.second; ++vi.first) {
      VertexPtr vp = getVertex(*vi.first);
      if (vp->isValue() && vp->isOptimized()) {
        PoseValuePtr p = std::dynamic_pointer_cast<value::Pose>(vp);
        if (p) {
          p->setPose(optimizer_->getPose(p->getKey()));
        }
      }
    }
  }

  std::string Graph::info(BoostGraphVertex v) const {
    VertexConstPtr  vp = graph_[v].vertex;
    return (vp->getLabel());
  }

  // static method!
  Graph::Id Graph::make_id(const ros::Time &t, const std::string &name) {
    return (name + "_" + std::to_string(t.toNSec()));
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
