/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/boost_graph.h"
#include "tagslam/body.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#pragma once

namespace tagslam {
  class Graph {
  public:
    Graph();
    ~Graph();
    void startUpdate();
    void endUpdate();
    void addBody(const Body &body);
    BoostGraph &getSubGraph() { return (subGraph_); }
  private:
    bool findOptimizationCandidates(const BoostGraph &graph, BoostGraph *subGraph,
                                    const std::vector<BoostGraph::vertex_descriptor> &factors);
    BoostGraph graph_;
    BoostGraph subGraph_;
  };
}
