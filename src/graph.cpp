/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/factor/absolute_pose_prior.h"
#include "tagslam/factor/relative_pose_prior.h"
#include "tagslam/value/pose.h"
#include "tagslam/optimizer.h"

#include <boost/range/irange.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_utility.hpp>
#include <vector>
#include <fstream>
#include <queue>
#include <map>
#include <sstream>


namespace tagslam {

  using boost::irange;

  template <class G>
  class LabelWriter {
  public:
    LabelWriter(const G &g) : graph(&g) { }
    template <class VertexOrEdge>
    void operator()(std::ostream &out, const VertexOrEdge& v) const {
      out << "[label=\"" << (*graph)[v].vertex->getLabel() << "\", shape="
          << (*graph)[v].vertex->getShape() <<"]";
    }
  private:
    const G *graph;
  };

  template<class G>
  static void plot(const std::string &fname, const G &graph) {
    std::ofstream ofile(fname);
    boost::write_graphviz(ofile, graph, LabelWriter<G>(graph));
  }
/*
  class demo_visitor : public boost::default_bfs_visitor {
  public:
    template <typename Vertex, typename Graph>
    void discover_vertex(Vertex u, Graph &g) {
      printf("Visited vertex %lu with name %s\n",   u, g[u].name.c_str());
    };
  };
*/
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
    if (optimizer_) {
      optimizer_->optimize();
    }
  }

  void Graph::addBody(const Body &body) {
    while ((int) bodyLookupTable_.size() <= body.getId()) {
      bodyLookupTable_.push_back(Entry());
    }
    std::list<BoostGraphVertex> optValues, optFactors;
    // add body pose as vertex
    std::shared_ptr<value::Pose>
      bv(new value::Pose(ros::Time(0), body.getPoseWithNoise().getPose(),
                         "body:" + body.getName()));
    BoostGraphVertex bodyVertex =
      boost::add_vertex(GraphVertex(bv), graph_);
    bodyLookupTable_[body.getId()] = Entry(ros::Time(0), bodyVertex, bv);
    if (body.isStatic()) {
      std::shared_ptr<Vertex> pv(
        new factor::AbsolutePosePrior(ros::Time(0), body.getPoseWithNoise(),
                                      "body:" + body.getName()));
      BoostGraphVertex priorVertex =
        boost::add_vertex(GraphVertex(pv), graph_);
      boost::add_edge(bodyVertex, priorVertex, GraphEdge(0), graph_);
      // if we have a pose prior, we can throw it into the optimzer
      // right away
      optValues.push_back(bodyVertex);
      optFactors.push_back(priorVertex);
    }

    // add associated tags as vertices, also create
    // edges if pose estimate is available
    for (const auto &tag: body.getTags()) {
      // add tag vertex to the graph
      BoostGraphVertex tagVertex = addTag(*tag);
      
      // add tag prior edge if known
      if (tag->getPoseWithNoise().isValid()) {
        // first add prior factor
        std::shared_ptr<Vertex> ptv(
          new factor::AbsolutePosePrior(ros::Time(0), tag->getPoseWithNoise(),
                                        "tag:"+std::to_string(tag->getId())));
        BoostGraphVertex tagPriorVertex =
          boost::add_vertex(GraphVertex(ptv), graph_);
        // then add edge to it
        boost::add_edge(tagPriorVertex, tagVertex, GraphEdge(1), graph_);
        optValues.push_back(tagVertex);
        optFactors.push_back(tagPriorVertex);
      }
      
    }
    ROS_INFO_STREAM("added body " << body.getName() << " with "
                    << body.getTags().size() << " tags");

    // must first add values so keys are available!
    for (const auto &v: optValues) {
      graph_[v].vertex->addToOptimizer(optimizer_, v, &graph_);
    }
    for (const auto &v: optFactors) {
      graph_[v].vertex->addToOptimizer(optimizer_, v, &graph_);
    }
  }

  BoostGraphVertex
  Graph::addTag(const Tag2 &tag) {
    if (tagIdToPoseVertex_.count(tag.getId()) != 0) {
      ROS_ERROR_STREAM("duplicate tag id: " << tag.getId());
      throw std::runtime_error("duplicate tag id!");
    }
    const std::string nm = "tag:" + std::to_string(tag.getId());
    std::shared_ptr<Vertex>
      vt(new value::Pose(ros::Time(0), tag.getPoseWithNoise().getPose(), nm));
    BoostGraphVertex tagVertex =
      boost::add_vertex(GraphVertex(vt), graph_);
    tagIdToPoseVertex_[tag.getId()] = tagVertex;
    return (tagVertex);
  }

  bool
  Graph::getBodyPose(const ros::Time &t,
                     const BodyConstPtr &body, Transform *tf) const {
    const auto &entry = bodyLookupTable_[body->getId()];
    if (entry.time != t && entry.time != ros::Time(0)) {
      ROS_INFO_STREAM(body->getName() << " no valid time entry for " << t);
      return (false);
    }
    if (entry.pose->getKey() == 0) {
      ROS_INFO_STREAM(body->getName() << " no valid pose for " << t);
      return (false);
    }
    *tf = optimizer_->getPose(entry.pose->getKey());
    return (true);
  }

  bool Graph::getTagPose(const ros::Time &t,
                         const Tag2ConstPtr &tag, Transform *tf) const {
    const auto it = tagIdToPoseVertex_.find(tag->getId());
    if (it == tagIdToPoseVertex_.end()) {
      return (false);
    }
    // find vertex and convert it to pose
    const GraphVertex &v = graph_[it->second];
    std::shared_ptr<value::Pose> p =
      std::dynamic_pointer_cast<value::Pose>(v.vertex);
    if (!p) {
      ROS_ERROR_STREAM("vertex for tag " << tag->getId() << " no pose!");
      return (false);
    }
    if (!p->isOptimized()) {
      return (false);
    }
    *tf = optimizer_->getPose(p->getKey());
    return (true);
  }

  void
  Graph::test() {
  }


  PoseValuePtr
  Graph::addPoseWithPrior(const ros::Time &t, const std::string &name,
                          const PoseWithNoise &pn,
                          BoostGraphVertex *vd) {
    // new pose value
    PoseValuePtr pvp(new value::Pose(t, pn.getPose(), name, true));
    // add new pose value to the optimizer
    pvp->setKey(optimizer_->addPose(pvp->getPose()));
    // add new pose value into the boost graph
    *vd = boost::add_vertex(GraphVertex(pvp), graph_);

    // 
    std::shared_ptr<factor::AbsolutePosePrior>
      fac(new factor::AbsolutePosePrior(t, pn, name));
    // add factor to optimizer
    fac->setKey(optimizer_->addAbsolutePosePrior(pvp->getKey(), pn));
    // add factor vertex to boost graph
    BoostGraphVertex fv = boost::add_vertex(GraphVertex(fac), graph_);
      
    // now add edge between factor and value
    boost::add_edge(fv, *vd, GraphEdge(0), graph_);
    return (pvp);
  }

  void
  Graph::addBodyPoseDelta(const ros::Time &tPrev, const ros::Time &tCurr,
                          const BodyConstPtr &body,
                          const PoseWithNoise &deltaPose) {
    auto &entry = bodyLookupTable_[body->getId()];
    if (entry.time == tCurr) {
      ROS_ERROR_STREAM("ign dup pose " << tCurr  << " " << body->getName());
      return;
    }
    if (entry.time == ros::Time(0)) {
#define INIT_POSE_WITH_IDENTITY     
#ifdef INIT_POSE_WITH_IDENTITY
      // -------- update entry
      PoseWithNoise pn(Transform::Identity(), PoseNoise2::make(0.1, 0.1), true);
      entry.time   = tPrev;
      entry.pose   = addPoseWithPrior(tPrev, "body:" +
                                      body->getName(), pn, &entry.vertex);

      ROS_INFO_STREAM("added initial pose for t: " << entry.time << " valid: " << entry.pose->isValid());
#else
      PoseValuePtr pvp(new value::Pose(tPrev, Transform::Identity(),
                                       "body:" + body->getName(), false));
      entry.time   = tPrev;
      entry.vertex = boost::add_vertex(GraphVertex(pvp), graph_);
      entry.pose   = pvp;
#endif
    }
    PoseValuePtr pp = entry.pose;
 
    const Transform newPose = deltaPose.getPose() * pp->getPose();
    // note that the new pose is set to invalid if the previous one is
    PoseValuePtr np(new value::Pose(tCurr, newPose,
                                    "body:" + body->getName(), pp->isValid()));
    // add new pose value to graph
    const BoostGraphVertex npv = boost::add_vertex(GraphVertex(np), graph_);

    // add new factor and its edges to graph
    VertexPtr fvv(new factor::RelativePosePrior(tCurr, deltaPose,
                                               "body:" + body->getName()));
    const BoostGraphVertex fv = boost::add_vertex(GraphVertex(fvv), graph_);
    boost::add_edge(fv, entry.vertex, GraphEdge(0), graph_);
    boost::add_edge(fv, npv, GraphEdge(1), graph_);
    
    // if poses are valid, add to optimizer
    if (pp->isValid()) {
      ROS_INFO_STREAM("previous pose is valid!");
      if (pp->getKey() == 0) {
        ROS_INFO_STREAM("adding original pose!");
        // no valid optimizer key -> pose value not entered yet!
        pp->setKey(optimizer_->addPose(pp->getPose()));
      }
      const ValueKey key1  = pp->getKey();
      ROS_INFO_STREAM("adding new pose!");
      np->setKey(optimizer_->addPose(newPose));
      optimizer_->addRelativePosePrior(key1, np->getKey(), deltaPose);
    } else {
      ROS_INFO_STREAM("previous pose is invalid!");
    }
    entry.time   = tCurr;
    entry.vertex = npv; // new pose value becomes previous...
    entry.pose   = np;
  }


  Graph::~Graph() {
  }

  void Graph::plotDebug(const ros::Time &t, const std::string &tag) {
    std::stringstream ss;
    ss << tag << "_" <<  t.toNSec() << ".dot";
    plot(ss.str(), graph_);
  }
}  // end of namespace
