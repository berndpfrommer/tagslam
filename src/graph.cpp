/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/factor/absolute_pose_prior.h"
#include "tagslam/factor/relative_pose_prior.h"
#include "tagslam/value/pose.h"


#include <boost/range/irange.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_utility.hpp>
#include <vector>
#include <fstream>
#include <queue>
#include <map>

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

  class demo_visitor : public boost::default_bfs_visitor {
  public:
    template <typename Vertex, typename Graph>
    void discover_vertex(Vertex u, Graph &g) {
      printf("Visited vertex %lu with name %s\n",   u, g[u].name.c_str());
    };
  };

  Graph::Graph() {
#if 0    
    std::vector<BoostGraph::vertex_descriptor> vertices;
    for (const int i: irange(0, 10)) {
      BoostGraph::vertex_descriptor v =
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
    typedef std::pair<BoostGraph::vertex_descriptor, BoostGraph::vertex_descriptor> QueueEntry;
    typedef std::queue<QueueEntry> Queue;
    typedef BoostGraph::out_edge_iterator OutEdgeIterator;
    SubGraphMaker(const BoostGraph &graph, BoostGraph *sg) :
      graph_(graph), subGraph_(sg) {}

    void add(const std::vector<BoostGraph::vertex_descriptor> &startVertices) {
      Queue().swap(openQueue_); // clears queue
      foundVertices_.clear();
      foundEdges_.clear();
      for (const auto &startVertex: startVertices) {
        add(startVertex);
      }
    }

    void add(const BoostGraph::vertex_descriptor &start) {
      if (foundVertices_.count(start) != 0) {
        return; // already in graph, we're done
      }
      // add start vertex to subgraph and push it into the queue
      BoostGraph::vertex_descriptor u = boost::add_vertex(graph_[start], *subGraph_);
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
          BoostGraph::vertex_descriptor src(boost::source(*it, graph_)),
            targ(boost::target(*it, graph_));
          
          if (foundVertices_.count(targ) == 0) {
            BoostGraph::vertex_descriptor v = boost::add_vertex(graph_[targ], *subGraph_);
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
    std::set<BoostGraph::vertex_descriptor> foundVertices_;
    std::map<BoostGraph::vertex_descriptor, BoostGraph::vertex_descriptor> vertexMap_;
    std::set<BoostGraph::edge_descriptor>   foundEdges_;

  };


  void Graph::startUpdate() {
    subGraph_ = BoostGraph(); // clear graph
  }

  void Graph::endUpdate() {
    plot<BoostGraph>("full.dot", graph_);
    plot<BoostGraph>("subgraph.dot", subGraph_);
  }

  bool
  Graph::findOptimizationCandidates(const BoostGraph &graph, BoostGraph *subGraph,
                                    const std::vector<BoostGraph::vertex_descriptor> &factors) {
#if 0    
    UnoptimizedValuesPredicate<BoostGraph> filter(graph);
    typedef boost::filtered_graph<
      BoostGraph, boost::keep_all, UnoptimizedValuesPredicate<BoostGraph>> FilteredGraph;
    FilteredGraph fg(graph, boost::keep_all(), filter);
    plot<FilteredGraph>("filtered.dot", fg);
#endif
    SubGraphMaker sgm(graph, subGraph);
    sgm.add(factors);
    return (true);
  }

  void Graph::addBody(const Body &body) {
    // add body pose as vertex
    std::shared_ptr<Vertex> bv(new value::Pose(ros::Time(0), body.getPoseWithNoise(),
                                               "body:" + body.getName()));
    BoostGraph::vertex_descriptor bodyVertex =
      boost::add_vertex(GraphVertex(bv), graph_);

    std::vector<BoostGraph::vertex_descriptor> factors;
    if (body.getIsStatic()) {
      std::shared_ptr<Vertex> pv(
        new factor::AbsolutePosePrior(ros::Time(0), body.getPoseWithNoise(),
                                      "body:" + body.getName()));
      BoostGraph::vertex_descriptor priorVertex =  boost::add_vertex(GraphVertex(pv), graph_);
      factors.push_back(priorVertex);
      boost::add_edge(bodyVertex, priorVertex, GraphEdge(0), graph_);
    }

    // add associated tags as vertices, also create
    // edges if pose estimate is available
    for (const auto &tag: body.getTags()) {
      // add tag vertex
      const std::string name = "tag:" + std::to_string(tag->getId());
      std::shared_ptr<Vertex> vt(new value::Pose(ros::Time(0), PoseWithNoise(), name));
      BoostGraph::vertex_descriptor tagVertex =
        boost::add_vertex(GraphVertex(vt), graph_);
      
      // add tag prior edge if known
      if (tag->getPoseWithNoise().isValid()) {
        // first add prior factor
        std::shared_ptr<Vertex> ptv(
          new factor::RelativePosePrior(ros::Time(0), tag->getPoseWithNoise(),
                                        "tag:" + std::to_string(tag->getId())));
        BoostGraph::vertex_descriptor tagPriorVertex =
          boost::add_vertex(GraphVertex(ptv), graph_);
        factors.push_back(tagPriorVertex);
        // then add edges to it
        boost::add_edge(bodyVertex, tagPriorVertex, GraphEdge(0), graph_);
        boost::add_edge(tagPriorVertex, tagVertex, GraphEdge(1), graph_);
      }
      
    }
    ROS_INFO_STREAM("added body " << body.getName() << " with "
                    << body.getTags().size() << " tags");
    findOptimizationCandidates(graph_, &subGraph_, factors);
  }

  

  Graph::~Graph() {
  }


}  // end of namespace
