/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/factor/absolute_pose_prior.h"
#include "tagslam/factor/relative_pose_prior.h"
#include "tagslam/factor/tag_projection.h"
#include "tagslam/value/pose.h"
#include "tagslam/optimizer.h"
#include "tagslam/pnp.h"

#include <boost/range/irange.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_utility.hpp>
#include <vector>
#include <algorithm>
#include <fstream>
#include <queue>
#include <map>
#include <sstream>

//#define DEBUG_GRAPH

namespace tagslam {

  using boost::irange;

  static Graph::Id make_id(const ros::Time &t, const std::string &name) {
    return (name + "_" + std::to_string(t.toNSec()));
  }

  static double find_size_of_tag(const Eigen::Matrix<double, 4, 2> &x) {
    // shoelace formula
    const double A = fabs(x(0,0)*x(1,1) + x(1,0)*x(2,1) + x(2,0)*x(3,1) + x(3,0)*x(0,1)
                          -x(1,0)*x(0,1) - x(2,0)*x(1,1) - x(3,0)*x(2,1) - x(0,0)*x(3,1));
    return (A);
  }
  static bool contains(std::list<BoostGraphVertex> &c,
                       const BoostGraphVertex &v) {
    return (std::find(c.begin(), c.end(), v) != c.end());
  }
  static bool contains(std::vector<BoostGraphVertex> &c,
                       const BoostGraphVertex &v) {
    return (std::find(c.begin(), c.end(), v) != c.end());
  }
  

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
#if 0    
    std::vector<BoostGraphVertex> vertices;
    for (const int i: irange(0, 10)) {
      BoostGraphVertex v =
        boost::add_vertex(Vertex("name_" + std::to_string(i)), graph_);
      vertices.push_back(v);
    }
    for (const int i: irange(1, 5)) {
      boost::add_edge(vertices[0], vertices[i], graph_);
    }
    for (const int i: irange(5, 10)) {
      boost::add_edge(vertices[4], vertices[i], graph_);
    }
    demo_visitor vis;
    //boost::breadth_first_search(graph_, vertices[4], boost::visitor(vis));

#endif
  }

  Graph::~Graph() {
    std::cout << profiler_ << std::endl;
    std::cout.flush();
  }
/*
  template <class G>
  struct UnoptimizedValuesPredicate {
    UnoptimizedValuesPredicate() : graph(NULL) {}
    UnoptimizedValuesPredicate(const G &g) : graph(&g) {}
    template <typename V>
    bool operator()(const V &v) const {
      return (!(*graph)[v].vertex->getIsOptimized());
    }
    const G *graph;
  };


  class SubGraphMaker {
  public:
    typedef std::pair<BoostGraphVertex,
                      BoostGraphVertex> QueueEntry;
    typedef std::queue<QueueEntry> Queue;
    typedef BoostGraph::out_edge_iterator OutEdgeIterator;
    SubGraphMaker(const BoostGraph &graph, BoostGraph *sg) :
      graph_(graph), subGraph_(sg) {}

    void add(const std::vector<BoostGraphVertex> &startVertices) {
      Queue().swap(openQueue_); // clears queue
      foundVertices_.clear();
      foundEdges_.clear();
      for (const auto &startVertex: startVertices) {
        add(startVertex);
      }
    }

    void add(const BoostGraphVertex &start) {
      if (foundVertices_.count(start) != 0) {
        return; // already in graph, we're done
      }
      // add start vertex to subgraph and push it into the queue
      BoostGraphVertex u = boost::add_vertex(graph_[start], *subGraph_);
      vertexMap_[start] = u;
      // have to remember vertices in both graphs to be
      // able to later make the edges
      openQueue_.push(QueueEntry(start, u));
      while (!openQueue_.empty()) {
        Queue::value_type q = openQueue_.front();
        // iterate over all adjacent vertices, insert them
        // in the subgraph and to the back of the queue
        OutEdgeIterator it, itEnd;
        std::tie(it, itEnd) = boost::out_edges(q.first, graph_);
        for (; it != itEnd; ++it) {
          // can do more filtering here if desired
          BoostGraphVertex src(boost::source(*it, graph_)),
            targ(boost::target(*it, graph_));
          
          if (foundVertices_.count(targ) == 0) {
            BoostGraphVertex v = boost::add_vertex(graph_[targ], *subGraph_);
            vertexMap_[targ] = v;
            openQueue_.push(QueueEntry(src, targ));
            foundVertices_.insert(targ);
          }
          // at this point we can be sure that both source and target
          // vertices are in the subgraph
          const auto &st = vertexMap_[targ];
          if (!boost::edge(q.second, st, *subGraph_).second) {
            boost::add_edge(q.second, st, graph_[*it], *subGraph_);
          }
        }
        openQueue_.pop();
      }
    }
  private:
    const BoostGraph                       &graph_;
    BoostGraph                             *subGraph_;
    Queue                                   openQueue_;
    std::set<BoostGraphVertex> foundVertices_;
    std::map<BoostGraphVertex, BoostGraphVertex> vertexMap_;
    std::set<BoostGraph::edge_descriptor>   foundEdges_;

  };

*/  

  void Graph::optimize() {
    profiler_.reset();
    if (optimizer_) {
      if (optimizeFullGraph_) {
        optimizer_->optimizeFullGraph();
      } else {
        optimizer_->optimize();
      }
    }
    profiler_.record("optimize");
  }

  void Graph::addBody(const Body &body) {
    // add body pose as vertex
    if (body.isStatic()) {
      const ros::Time t0(0);
      if (body.getPoseWithNoise().isValid()) {
        Graph::VertexPose vp = 
          addPoseWithPrior(t0, body_name(body.getName()),
                           body.getPoseWithNoise());
      }
    } 
    // add associated tags as vertices
    for (const auto &tag: body.getTags()) {
      addTag(*tag);
    }
    ROS_INFO_STREAM("added body " << body.getName() << " with "
                    << body.getTags().size() << " tags");
  }

  Graph::VertexPose
  Graph::addTag(const Tag2 &tag) {
    const string name = tag_name(tag.getId());
    const ros::Time t0(0);
    if (tag.getPoseWithNoise().isValid()) {
      return (addPoseWithPrior(t0, name, tag.getPoseWithNoise()));
    } else {
      return (addPose(t0, name, tag.getPoseWithNoise().getPose(), false));
    }
  }

  bool Graph::getPose(const ros::Time &t, const string &name,
                      Transform *tf) const {
    VertexPose vp = findPose(t, name);
    if (vp.pose && vp.pose->isOptimized()) {
      *tf = optimizer_->getPose(vp.pose->getKey());
      return (true);
    }
    return (false);
  }

  Graph::VertexPose
  Graph::findPose(const ros::Time &t, const string &name) const {
    const auto it = idToVertex_.find(make_id(t, name));
    if (it == idToVertex_.end()) {
      return (VertexPose()); // not found, return empty vp
    }
    const GraphVertex &v = graph_[it->second];
    std::shared_ptr<value::Pose> p =
      std::dynamic_pointer_cast<value::Pose>(v.vertex);
    if (!p) {
      ROS_ERROR_STREAM("vertex for id " << name << " is no pose!");
      return (VertexPose());
    }
    return (VertexPose(it->second, p));
  }

  void
  Graph::test() {
  }


  BoostGraphVertex
  Graph::addPrior(const ros::Time &t, const VertexPose &vp,
                  const string &name,
                  const PoseWithNoise &pn) {
    if (!vp.pose->isOptimized()) {
      vp.pose->setPose(pn.getPose());
      vp.pose->setKey(optimizer_->addPose(pn.getPose()));
    }
    std::shared_ptr<factor::AbsolutePosePrior>
      fac(new factor::AbsolutePosePrior(t, pn, name));
    // add factor to optimizer
    fac->setKey(optimizer_->addAbsolutePosePrior(vp.pose->getKey(), pn));
    // add factor vertex to boost graph
    BoostGraphVertex fv = boost::add_vertex(GraphVertex(fac), graph_);
    // now add edge between factor and value
    boost::add_edge(fv, vp.vertex, GraphEdge(0), graph_);
    return (fv);
  }
  
  Graph::VertexPose
  Graph::addPoseWithPrior(const ros::Time &t, const string &name,
                          const PoseWithNoise &pn) {
    // add pose into boost graph
    VertexPose vp = addPose(t, name, pn.getPose(), true);
    // add pose value to optimizer
    vp.pose->setKey(optimizer_->addPose(vp.pose->getPose()));

    addPrior(t, vp, name, pn);
    return (vp);
  }

  Graph::VertexPose
  Graph::addPose(const ros::Time &t, const string &name,
                 const Transform &pose, bool poseIsValid) {
    Id id = make_id(t, name);
    if (hasId(id)) {
      ROS_ERROR_STREAM("duplicate pose added, id: " << id);
      throw std::runtime_error("duplicate pose added!");
    }

    PoseValuePtr np(new value::Pose(t, pose, name, poseIsValid));
    // add new pose value to graph
    const BoostGraphVertex npv = boost::add_vertex(GraphVertex(np), graph_);
    idToVertex_.insert(IdToVertexMap::value_type(id, npv));
    return (VertexPose(npv, np));
  }

  BoostGraphVertex
  Graph::addBodyPoseDelta(const ros::Time &tPrev, const ros::Time &tCurr,
                          const BodyConstPtr &body,
                          const PoseWithNoise &deltaPose) {
    Transform prevPose;
    string name   = body_name(body->getName());
    VertexPose pp = findPose(tPrev, name);
    VertexPose cp = findPose(tCurr, name);
    if (!pp.pose) {
      ROS_DEBUG_STREAM("adding previous pose for " << name << " " << tPrev);
      pp = addPose(tPrev, name, Transform::Identity(), false);
    }
    if (!cp.pose) {
      ROS_DEBUG_STREAM("adding current pose for " << name << " " << tCurr);
      cp = addPose(tCurr, name, Transform::Identity(), false);
    }
    return (addRelativePosePrior(tCurr, "bodyrel:" + body->getName(),
                                 pp, cp, deltaPose, false));
  }

  BoostGraphVertex
  Graph::addRelativePosePrior(const ros::Time &t,
                              const string &name,
                              const VertexPose &vp1,
                              const VertexPose &vp2,
                              const PoseWithNoise &deltaPose,
                              bool addToOptimizer) {
    // add new factor and its edges to graph
    FactorPtr fvv(new factor::RelativePosePrior(t, deltaPose, name));
    const BoostGraphVertex fv = boost::add_vertex(GraphVertex(fvv), graph_);
    boost::add_edge(fv, vp1.vertex, GraphEdge(0), graph_);
    boost::add_edge(fv, vp2.vertex, GraphEdge(1), graph_);
    
    if (addToOptimizer) {
      fvv->setIsValid(true);
      fvv->setKey(optimizer_->addRelativePosePrior(
                    vp1.pose->getKey(), vp2.pose->getKey(),
                    deltaPose));
    }
    ROS_DEBUG_STREAM("adding " << fvv->getLabel() << " conn: " << vp1.pose->getLabel() << " -> " << vp2.pose->getLabel());
    return (fv);
  }

  BoostGraphVertex
  Graph::addProjectionFactor(const ros::Time &t,
                             const Tag2ConstPtr &tag,
                             const Camera2ConstPtr &cam,
                             const geometry_msgs::Point *imgCorners) {
    BoostGraphVertex v;
    // connect: tag_body_pose, tag_pose, cam_pose, rig_pose
    VertexPose tagvp = findPose(ros::Time(0),
                                Graph::tag_name(tag->getId()));
    if (tagvp.pose == NULL) {
      ROS_ERROR_STREAM("no pose found for tag: " << tag->getId());
      throw std::runtime_error("no tag pose found!");
    }
    BodyConstPtr body = tag->getBody();
    VertexPose bodyvp = findPose(body->isStatic() ? ros::Time(0) : t,
                                 Graph::body_name(body->getName()));
    if (bodyvp.pose == NULL) {
      ROS_ERROR_STREAM("no pose found for body: " << body->getName());
      throw std::runtime_error("no body pose found!");
    }
    BodyConstPtr rig = cam->getRig();
    VertexPose rigvp = findPose(rig->isStatic() ? ros::Time(0) : t,
                                Graph::body_name(rig->getName()));
    if (rigvp.pose == NULL) {
      ROS_ERROR_STREAM("no pose found for rig: " << rig->getName());
      throw std::runtime_error("no rig pose found!");
    }
    VertexPose camvp = findPose(ros::Time(0), Graph::cam_name(cam->getName()));
    if (camvp.pose == NULL) {
      ROS_ERROR_STREAM("no pose found for cam: " << cam->getName());
      throw std::runtime_error("no cam pose found!");
    }

    VertexPtr pjfac(
      new factor::TagProjection(t, cam, tag, imgCorners, cam->getName() +
                                "-" + std::to_string(tag->getId())));
    v = boost::add_vertex(GraphVertex(pjfac), graph_);
    boost::add_edge(v, camvp.vertex,  GraphEdge(0), graph_); // T_r_c
    boost::add_edge(v, rigvp.vertex,  GraphEdge(1), graph_); // T_w_r
    boost::add_edge(v, bodyvp.vertex, GraphEdge(2), graph_); // T_w_b
    boost::add_edge(v, tagvp.vertex,  GraphEdge(3), graph_); // T_b_o
    return (v);
  }
#if 0 
  class demo_visitor : public boost::default_bfs_visitor {
  public:
    template <typename V, typename Graph>
    void discover_vertex(V u, Graph &g) {
      VertexConstPtr vt = g[u].vertex;
      ValueConstPtr vp = std::dynamic_pointer_cast<const value::Value>(vt);
      bool isValid = vp && vp->isValid();
      if (!vp) {
        ROS_INFO_STREAM(" factor: " << u << " - " << vt->getLabel());
        // iterate through all edges
        // - if sufficient number of values of edges are known (ESTABLISHED, VALID, OPTIMIZED),
        //   add factor and values to subgraph.
        //   (unless there). Mark the newly established value vertex as ESTABLISHED
        //   (use property map for that?) and visit
        //   all the newly established value vertex'es factors
        // - if insufficient number of values are known, don't pursue any of the edges further

      } else {
        ROS_INFO_STREAM("  value: " << u << " - " << vt->getLabel() << (isValid ? " VALID" : " UNKNOWN"));
      }
    };
  };
#endif

  void
  Graph::examine(const ros::Time &t, BoostGraphVertex fac,
                 std::list<BoostGraphVertex> *factorsToExamine,
                 SubGraph *found, SubGraph *sg) {
    // This is the factor vertex we are checking
    VertexConstPtr fv = graph_[fac].vertex;
    ROS_DEBUG_STREAM("examining factor: " << fv->getLabel());
    // find out if this factor allows us to determine a new value
    auto edges = boost::out_edges(fac, graph_);
    int numEdges(0), numValid(0);
    BoostGraphVertex valueVertex;
    std::list<BoostGraphVertex> values;
    for (auto edgeIt = edges.first; edgeIt != edges.second; ++edgeIt) {
      BoostGraphVertex vv  = boost::target(*edgeIt, graph_); // value vertex
      VertexConstPtr   vvp = graph_[vv].vertex; // pointer to value
      ValueConstPtr vp = std::dynamic_pointer_cast<const value::Value>(vvp);
      numEdges++;
      if ((vp && vp->isValid()) || found->values.count(vv) != 0) {
        //ROS_INFO_STREAM(" has valid    value: " << vp->getLabel());
        numValid++;
      } else {
        //ROS_INFO_STREAM(" has no valid value: " << vp->getLabel());
        valueVertex = vv;
      }
      values.push_back(vv);
    }
    if (numValid == numEdges - 1) {
      // establishes new value, let's explore the new
      VertexConstPtr vt = graph_[valueVertex].vertex;
      ValueConstPtr vp = std::dynamic_pointer_cast<const value::Value>(vt);
      ROS_DEBUG_STREAM(" factor establishes new value: " << vp->getLabel());
      auto &ff = found->factors;
      if (std::find(ff.begin(), ff.end(), fac) == ff.end()) {
        ff.push_back(fac);
        sg->factors.push_back(fac);
      }
      for (const auto vv: values) {
        sg->values.insert(vv);
        //ROS_INFO_STREAM("  adding new corresponding values: " << info(vv));
        found->values.insert(vv);
      }
      
      auto edges = boost::out_edges(valueVertex, graph_);
      for (auto edgeIt = edges.first; edgeIt != edges.second; ++edgeIt) {
        BoostGraphVertex fv = boost::target(*edgeIt, graph_); // factor vertex
        VertexConstPtr  fvp = graph_[fv].vertex; // pointer to factor
        FactorConstPtr   fp = std::dynamic_pointer_cast<const factor::Factor>(fvp);
        if (fp) {
          if (fv != fac) { // no connections back
            ROS_DEBUG_STREAM("  " << vp->getLabel() << " activates " << fp->getLabel());
            if (fp->getTime() == t || fp->getTime() == ros::Time(0)) {
              factorsToExamine->push_front(fv);
            } else {
              TimeToVertexesMap::iterator it = times_.find(fp->getTime());
              if (it == times_.end()) {
                ROS_DEBUG_STREAM("   first vertex at time: " << fp->getTime());
                it = times_.insert(TimeToVertexesMap::value_type(fp->getTime(), std::vector<BoostGraphVertex>())).first;
              }
              auto &c = it->second;
              if (!contains(c, fv)) {
                ROS_DEBUG_STREAM("   remembering factor " << fp->getLabel());
                c.push_back(fv);
              } else {
                ROS_DEBUG_STREAM("   already remembered factor " << fp->getLabel());
              }
            }
          }
        }
      }
    } else if (numValid == numEdges) {
      // this factor does not establish a new value, but
      // provides an additional measurement on existing ones.
      ROS_DEBUG_STREAM(" factor provides additional measurement: " << fv->getLabel());
      auto &ff = found->factors;
      if (std::find(ff.begin(), ff.end(), fac) == ff.end()) {
        ff.push_back(fac);
        sg->factors.push_back(fac);
      }
    } else {
      ROS_DEBUG_STREAM(" factor does not establish new values!");
    }
  }

  std::string Graph::info(BoostGraphVertex v) const {
    VertexConstPtr  vp = graph_[v].vertex;
    return (vp->getLabel());
  }

  std::vector<std::list<BoostGraphVertex>>
  Graph::findSubgraphs(const ros::Time &t,
                       const std::vector<BoostGraphVertex> &facs,
                       SubGraph *found) {
    profiler_.reset();
    std::vector<std::list<BoostGraphVertex>> sv;
    ROS_DEBUG_STREAM("======================= finding subgraphs for t = " << t);
    // first look over the new factors
    for (const auto &fac: facs) {
      ROS_DEBUG_STREAM(" ----- exploring new subgraph starting at: " << info(fac));
      if (!contains(found->factors, fac)) {
        // this is a new factor that has not been explored
        SubGraph sg;
        exploreSubGraph(t, fac, &sg, found);
        if (!sg.factors.empty()) {
          sv.push_back(sg.factors);    // transfer factors
        }
      }
    }
    profiler_.record("findSubGraphs");
    return (sv);
  }

  void
  Graph::processNewFactors(const ros::Time &t,
                           const std::vector<BoostGraphVertex> &facs) {
    ROS_DEBUG_STREAM("&&&&&&&&&&&&&&&&&&&&&&&&&&&&& got new factors for t = " << t);
    SubGraph found;
    std::vector<std::list<BoostGraphVertex>> sv;
    sv = findSubgraphs(t, facs, &found);
    if (sv.empty()) {
      ROS_INFO_STREAM("no new factors activated!");
      return;
    }
    initializeSubgraphs(sv);
    optimize();
    transferValues();
#if 0    
    for (const auto &kv: times_) {
      ROS_DEBUG_STREAM("time: " << kv.first);
    }
#endif
    std::cout << profiler_ << std::endl;
    ROS_DEBUG_STREAM("&-&-&-&-&-&-&-& done with new factors for t = " << t);
    while (!times_.empty()) {
      auto it = times_.rbegin();
      const ros::Time key = it->first;
      ROS_DEBUG_STREAM("++++++++++ handling old factors for t = " << key);
      sv = findSubgraphs(key, it->second, &found);
      initializeSubgraphs(sv);
      optimize();
      transferValues();
      // now remove factors that have been used
      ROS_DEBUG_STREAM("removing used factors...");
      for (auto ii = times_[key].begin(); ii != times_[key].end();) {
        if (contains(found.factors, *ii)) {
          ROS_DEBUG_STREAM("removing used factor " << info(*ii) << " for time "  << key);
          ii = times_[key].erase(ii);
        } else {
          ++ii;
        }
      }
      if (times_[key].empty()) {
        ROS_DEBUG_STREAM("all elements gone for time " << key);
        times_.erase(key);
      }
      std::cout << profiler_ << std::endl;
      std::cout.flush();
    }
  }

  void
  Graph::exploreSubGraph(const ros::Time &t,
                         BoostGraphVertex start,
                         SubGraph *subGraph, SubGraph *found) {
    std::list<BoostGraphVertex> factorsToExamine;
    factorsToExamine.push_back(start);
    while (!factorsToExamine.empty()) {
      BoostGraphVertex exFac = factorsToExamine.front();
      factorsToExamine.pop_front();
      // examine() may append new factors to factorsToExamine
      examine(t, exFac, &factorsToExamine, found, subGraph);
    }
  }

  void Graph::addProjectionFactorToOptimizer(const BoostGraphVertex v) {
    auto edges = boost::out_edges(v, graph_);
    std::vector<ValueKey> optKeys;
    for (auto edgeIt = edges.first; edgeIt != edges.second; ++edgeIt) {
      BoostGraphVertex vv  = boost::target(*edgeIt, graph_); // value vertex
      VertexPtr        vvp = graph_[vv].vertex; // pointer to value
      PoseValuePtr pp = std::dynamic_pointer_cast<value::Pose>(vvp);
      if (!pp) {
        ROS_ERROR_STREAM("vertex is no pose: " << vv << " " << info(vv));
        throw std::runtime_error("vertex is no pose");
      }
      if (!pp->isValid()) {
        ROS_ERROR_STREAM("vertex is not valid: " << info(vv));
        throw std::runtime_error("vertex is not valid");
      }
      if (!pp->isOptimized()) {
        ROS_DEBUG_STREAM("adding pose to optimizer: " << pp->getLabel());
        pp->setKey(optimizer_->addPose(pp->getPose()));
      }
      optKeys.push_back(pp->getKey());
    }
    BoostGraphVertex fv = boost::vertex(v, graph_); // factor vertex
    VertexPtr  fvp = graph_[fv].vertex; // pointer to factor
    TagProjectionFactorPtr fp =
      std::dynamic_pointer_cast<factor::TagProjection>(fvp);
    if (fp->isOptimized()) {
      ROS_ERROR_STREAM("factor already optimized: " << fp->getLabel());
      throw std::runtime_error("factor already optimized");
    }
    ROS_DEBUG_STREAM("adding tag projection factor to optimizer: " << info(v));
    fp->setKey(optimizer_->addTagProjectionFactor(fp->getImageCorners(),
                                                  fp->getTag()->getObjectCorners(),
                                                  fp->getCamera()->getName(),
                                                  fp->getCamera()->getIntrinsics(),
                                                  pixelNoise_,
                                                  optKeys[0], optKeys[1],
                                                  optKeys[2], optKeys[3]));
  }


  int
  Graph::findConnectedPoses(BoostGraphVertex v,
                            std::vector<PoseValuePtr> *poses) {
    auto edges = boost::out_edges(v, graph_);
    int missingIdx(-1), edgeNum(0), numMissing(0);
    poses->clear();
    for (auto edgeIt = edges.first; edgeIt != edges.second; ++edgeIt) {
      BoostGraphVertex vv  = boost::target(*edgeIt, graph_); // value vertex
      VertexPtr   vvp = graph_[vv].vertex; // pointer to value
      PoseValuePtr pp = std::dynamic_pointer_cast<value::Pose>(vvp);
      if (!pp) {
        ROS_ERROR_STREAM("vertex is no pose: " << vv);
        throw std::runtime_error("vertex is no pose");
      }
      poses->push_back(pp);
      //ROS_DEBUG_STREAM(" factor attached value: " << pp->getLabel());
      if (!pp->isValid()) {
        missingIdx = edgeNum;
        numMissing++;
        //ROS_DEBUG_STREAM(" missing value: " << pp->getLabel());
      } else {
        //ROS_DEBUG_STREAM(" valid value: " << pp->getLabel());
      }
      edgeNum++;
    }
    missingIdx = (numMissing == 1) ? missingIdx : -(numMissing + 1);
    return (missingIdx);
  }

  void
  Graph::setValueFromTagProjection(BoostGraphVertex v, const Transform &T_c_o)  {
    std::vector<PoseValuePtr> T;
    int idx = findConnectedPoses(v, &T);
    switch (idx) {
    case 0: { // T_r_c = T_r_w * T_w_b * T_b_o * T_o_c
      T[idx]->setPose(T[1]->getPose().inverse() * T[2]->getPose() *
                      T[3]->getPose() * T_c_o.inverse());
      break; }
    case 1: { // T_w_r = T_w_b * T_b_o * T_o_c * T_c_r
      T[idx]->setPose(T[2]->getPose() * T[3]->getPose() *
                      T_c_o.inverse() * T[0]->getPose().inverse());
      //ROS_DEBUG_STREAM("setting rig pose: " << T[idx]->getLabel() << std::endl << T[idx]->getPose());
      break; }
    case 2: { // T_w_b = T_w_r * T_r_c * T_c_o * T_o_b
      T[idx]->setPose(T[1]->getPose() * T[0]->getPose() *
                      T_c_o * T[3]->getPose().inverse());
      break; }
    case 3: { // T_b_o = T_b_w * T_w_r * T_r_c * T_c_o
      T[idx]->setPose(T[2]->getPose().inverse() * T[1]->getPose() *
                      T[0]->getPose() * T_c_o);
      break; }
    case -1: {
      // T_c_o = T_c_r[0] * T_r_w[1] * T_w_b[2] * T_b_o[3]
      Transform Test_c_o = T[0]->getPose().inverse() * T[1]->getPose().inverse() * T[2]->getPose() * T[3]->getPose();
      Transform poseDiff = T_c_o * Test_c_o.inverse();
      Eigen::AngleAxisd aa;
      aa.fromRotationMatrix(poseDiff.rotation());
      ROS_DEBUG_STREAM("dup factor with mismatch angle: " << aa.angle() << " len: " << poseDiff.translation().norm());
      return;
      break; }
    default: {
      ROS_DEBUG_STREAM("factor has multiple missing values!");
      return;
      break; }
    }
    ROS_DEBUG_STREAM("tag proj setting value of " << T[idx]->getLabel());
    ROS_DEBUG_STREAM("set pose: " << std::endl << T[idx]->getPose());
  }

  int
  Graph::setValueFromRelativePosePrior(BoostGraphVertex v, const Transform &deltaPose,
                                       std::vector<PoseValuePtr> *poses)  {
    std::vector<PoseValuePtr> &T = *poses;
    int idx = findConnectedPoses(v, &T);
    if (T.size()  != 2) {
      ROS_ERROR_STREAM("rel pose prior has wrong num connected: " << T.size());
      throw std::runtime_error("rel pose prior has wrong num conn");
    }
    switch (idx) {
    case 0: { // T_0 = T_1 * delta T^-1
      T[idx]->setPose(T[1]->getPose() * deltaPose.inverse());
      break; }
    case 1: { // T_1 = T_0 * delta T
      T[idx]->setPose(T[0]->getPose() * deltaPose);
      break; }
    case -1: {
      ROS_DEBUG_STREAM("delta pose factor has no missing values!");
      return (idx);
      break; }
    default: {
      ROS_DEBUG_STREAM("delta factor has multiple missing values!");
      return (idx);
      break; }
    }
    ROS_DEBUG_STREAM("rel pos setting value of " << T[idx]->getLabel());
    ROS_DEBUG_STREAM("set pose: " << std::endl << T[idx]->getPose());
    return (idx);
  }

                                               
  void Graph::initializeSubgraphs(const std::vector<std::list<BoostGraphVertex>> &verts) {
    profiler_.reset();
    ROS_DEBUG_STREAM("----------- initializing " << verts.size() << " subgraphs");
    for (const auto &vset: verts) {
      ROS_DEBUG_STREAM("---------- subgraph of size: " << vset.size());
      for (const auto &v: vset) {
        ROS_DEBUG_STREAM("    " << graph_[v].vertex->getLabel());
      }
      for (const auto &v: vset) {
        BoostGraphVertex fv = boost::vertex(v, graph_); // factor vertex
        VertexPtr  fvp = graph_[fv].vertex; // pointer to factor
        TagProjectionFactorPtr fp =
          std::dynamic_pointer_cast<factor::TagProjection>(fvp);
        if (fp) {
          // do homography for this vertex, add new values to graph and the optimizer!
          const CameraIntrinsics2 ci = fp->getCamera()->getIntrinsics();
          ROS_DEBUG_STREAM("computing pose for factor " << fvp->getLabel());
          auto tf = pnp::pose_from_4(fp->getImageCorners(),
                                     fp->getTag()->getObjectCorners(),
                                     ci.getK(), ci.getDistortionModel(), ci.getD());
          double sz = find_size_of_tag(fp->getImageCorners());
          ROS_DEBUG_STREAM("got homography, area " << sz << " good: " << tf.second << std::endl << tf.first);
          if (tf.second) {
            setValueFromTagProjection(fv, tf.first);
            if (!fp->isOptimized()) {
              // add factor and values to optimizer
              fp->setIsValid(true);
              addProjectionFactorToOptimizer(fv);
            }
          }
        } else {
          RelativePosePriorFactorPtr rp =
          std::dynamic_pointer_cast<factor::RelativePosePrior>(fvp);
          if (rp && !rp->isOptimized()) {
            std::vector<PoseValuePtr> poses;
            int idx = setValueFromRelativePosePrior(v, rp->getPoseWithNoise().getPose(), &poses);
            if (idx >= 0) {
              for (const auto i: irange(0, 2)) {
                if (!poses[i]->isOptimized()) {
                  poses[i]->setKey(optimizer_->addPose(poses[i]->getPose()));
                }
              }
              rp->setKey(optimizer_->addRelativePosePrior(
                           poses[0]->getKey(), poses[1]->getKey(),
                           rp->getPoseWithNoise()));
            }
          } else {
            ROS_DEBUG_STREAM("skipping factor: " << fvp->getLabel());
          }
        }
      }
    }
    profiler_.record("initializeSubgraphs");
  }

 
  void Graph::transferValues() {
    profiler_.reset();
    for (auto vi = boost::vertices(graph_); vi.first != vi.second; ++vi.first) {
      VertexPtr &vp = graph_[*vi.first].vertex;
      if (vp->isValue() && vp->isOptimized()) {
        PoseValuePtr p = std::dynamic_pointer_cast<value::Pose>(vp);
        if (p) {
          p->setPose(optimizer_->getPose(p->getKey()));
        }
      }
    }
    profiler_.record("transferValues");
  }

  void Graph::reoptimize() {
    optimizer_->optimizeFullGraph(true);
  }

  void Graph::plotDebug(const ros::Time &t, const string &tag) {
    std::stringstream ss;
    ss << tag << "_" <<  t.toNSec() << ".dot";
    plot(ss.str(), graph_);
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
