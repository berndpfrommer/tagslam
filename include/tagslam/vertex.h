/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <string>
#include <iostream>
#include "tagslam/boost_graph.h"

namespace tagslam {
 // need forward declaration here
  class GTSAMOptimizer;

  class Vertex {
  public:
    Vertex(const std::string &n = "", const std::string &s = ""):
      name(n), shape(s) {}
    virtual ~Vertex() {};
    // this virtual function needs to be implemented by all vertices
    virtual void addToOptimizer(GTSAMOptimizer *opt,
                                const BoostGraph::vertex_descriptor &v,
                                const BoostGraph *g) = 0;
    virtual bool isValue() const = 0;
    virtual std::string getLabel() const { return (name); }
    virtual std::string getShape() const { return (shape); }
    virtual bool  getIsOptimized() const { return (isOptimized); }
    void setIsOptimized(bool b) { isOptimized = b; }
    friend std::ostream &operator<<(std::ostream &os, const Vertex &v);
    // ----
    std::string name;
    std::string shape;
    bool        isOptimized{false};
  };
  std::ostream &operator<<(std::ostream &os, const Vertex &v);
}
