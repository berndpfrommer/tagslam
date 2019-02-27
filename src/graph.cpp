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

  static Graph::Id make_id(const ros::Time &t, const std::string &name) {
    return (name + "_" + std::to_string(t.toNSec()));
  }

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
      if (optimizeFullGraph_) {
        optimizer_->optimizeFullGraph();
      } else {
        optimizer_->optimize();
      }
    }
  }

  void Graph::addBody(const Body &body) {
    while ((int) bodyLookupTable_.size() <= body.getId()) {
      bodyLookupTable_.push_back(Entry());
    }
    // add body pose as vertex
    if (body.isStatic()) {
      const ros::Time t0(0);
      if (body.getPoseWithNoise().isValid()) {
        Graph::VertexPose vp = 
          addPoseWithPrior(t0, "body:"+body.getName(), body.getPoseWithNoise());
        bodyLookupTable_[body.getId()] = Entry(ros::Time(0), vp.vertex,vp.pose);
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
    const string name = "tag:" + std::to_string(tag.getId());
    const ros::Time t0(0);
    if (tag.getPoseWithNoise().isValid()) {
      return (addPoseWithPrior(t0, name, tag.getPoseWithNoise()));
    } else {
      return (addPose(t0, name, tag.getPoseWithNoise().getPose(), false));
    }
  }

  bool Graph::getPose(const ros::Time &t, const string &name,
                      Transform *tf) const {
    const auto it = idToVertex_.find(make_id(t, name));
    if (it == idToVertex_.end()) {
      return (false);
    }
    // find vertex and convert it to pose
    const GraphVertex &v = graph_[it->second];
    std::shared_ptr<value::Pose> p =
      std::dynamic_pointer_cast<value::Pose>(v.vertex);
    if (!p) {
      ROS_ERROR_STREAM("vertex for id " << name << " is no pose!");
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


  
  Graph::VertexPose
  Graph::addPoseWithPrior(const ros::Time &t, const string &name,
                          const PoseWithNoise &pn) {
    // add pose into boost graph
    VertexPose vp = addPose(t, name, pn.getPose(), true);
    // add pose value to optimizer
    vp.pose->setKey(optimizer_->addPose(vp.pose->getPose()));
   
    std::shared_ptr<factor::AbsolutePosePrior>
      fac(new factor::AbsolutePosePrior(t, pn, name));
    // add factor to optimizer
    fac->setKey(optimizer_->addAbsolutePosePrior(vp.pose->getKey(), pn));
    // add factor vertex to boost graph
    BoostGraphVertex fv = boost::add_vertex(GraphVertex(fac), graph_);
      
    // now add edge between factor and value
    boost::add_edge(fv, vp.vertex, GraphEdge(0), graph_);
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
      entry.time   = tPrev;
#define INIT_POSE_WITH_IDENTITY     
#ifdef INIT_POSE_WITH_IDENTITY
      PoseWithNoise pn(Transform::Identity(), PoseNoise2::make(0.1, 0.1), true);
      entry.vertexPose = addPoseWithPrior(tPrev, "body:" + body->getName(), pn);

      ROS_INFO_STREAM("added initial pose for t: " << entry.time <<
                      " valid: " << entry.vertexPose.pose->isValid());
#else
      entry.vertexPose = addPose(tPrev, "body:" + body->getName(),
                                 Transform::Identity(), false);
#endif
    }
    PoseValuePtr pp = entry.vertexPose.pose;
 
    // add new body pose to graph
    const Transform newPose = deltaPose.getPose() * pp->getPose();
    VertexPose nvp = addPose(tCurr, "body:" + body->getName(), newPose,
        pp->isValid());
  
    // add new factor and its edges to graph
    VertexPtr fvv(new factor::RelativePosePrior(tCurr, deltaPose,
                                               "body:" + body->getName()));
    const BoostGraphVertex fv = boost::add_vertex(GraphVertex(fvv), graph_);
    boost::add_edge(fv, entry.vertexPose.vertex, GraphEdge(0), graph_);
    boost::add_edge(fv, nvp.vertex, GraphEdge(1), graph_);
    
    // if poses are valid, add to optimizer
    if (pp->isValid()) {
      if (pp->getKey() == 0) {
        ROS_INFO_STREAM("adding original pose!");
        // no valid optimizer key -> pose value not entered yet!
        pp->setKey(optimizer_->addPose(pp->getPose()));
      }
      const ValueKey key1  = pp->getKey();
      ROS_INFO_STREAM("adding new pose!");
      nvp.pose->setKey(optimizer_->addPose(newPose));
      optimizer_->addRelativePosePrior(key1, nvp.pose->getKey(), deltaPose);
    } else {
      ROS_INFO_STREAM("previous pose is invalid!");
    }
    entry.time       = tCurr;
    entry.vertexPose = nvp; // new vertex/pose becomes previous...
  }

  void Graph::addTagMeasurements(
    const BodyVec &bodies,
    const BodyVec &nonstaticBodies,
    const std::vector<TagArrayConstPtr> &tagMsgs,
    const Camera2Vec &cameras) {
#if 0    
    for (const auto &b: nonstaticBodies) {
      // add empty position here
    }
    for (const auto i: irange(0ul, cameras_.size())) {
      graph_.addTagMeasurements(tagMsgs[i], cameras_[i]);
    }
#endif    
  }


  void Graph::plotDebug(const ros::Time &t, const string &tag) {
    std::stringstream ss;
    ss << tag << "_" <<  t.toNSec() << ".dot";
    plot(ss.str(), graph_);
  }
}  // end of namespace
