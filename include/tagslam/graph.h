/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph_vertex.h"
#include "tagslam/graph_edge.h"
#include "tagslam/body.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#pragma once

namespace tagslam {
  typedef boost::adjacency_list<
    boost::listS, boost::vecS, boost::undirectedS, GraphVertex, GraphEdge> BoostGraph;
  class Graph {
  public:
    Graph();
    ~Graph();
    void startUpdate();
    void endUpdate();
    void addBody(const Body &body);
  private:
    bool findOptimizationCandidates(const BoostGraph &graph, BoostGraph *subGraph,
                                    const std::vector<BoostGraph::vertex_descriptor> &factors);
    BoostGraph graph_;
    BoostGraph subGraph_;
  };
}
