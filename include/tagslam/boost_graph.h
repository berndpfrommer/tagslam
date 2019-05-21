/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "tagslam/graph_vertex.h"
#include "tagslam/graph_edge.h"


namespace tagslam {
  typedef  boost::adjacency_list<
    boost::listS, boost::vecS, boost::undirectedS,
    GraphVertex, GraphEdge> BoostGraph;
  typedef BoostGraph::vertex_descriptor BoostGraphVertex;
}
