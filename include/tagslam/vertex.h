/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/boost_graph.h"
#include <string>
#include <iostream>
#include <memory>
#include <ros/ros.h>

namespace tagslam {
  // need forward declaration here
  class Optimizer;
  class Vertex {
  public:
    Vertex(const std::string &n = "", const std::string &s = ""):
      name_(n), shape_(s) {}
    virtual ~Vertex() {};
    // this virtual function needs to be implemented by all vertices
    virtual void addToOptimizer(Optimizer *opt,
                                const BoostGraph::vertex_descriptor &v,
                                const BoostGraph *g) = 0;
    virtual bool isValue() const = 0;
    virtual std::string getLabel() const { return (name_); }
    virtual std::string getShape() const { return (shape_); }
    virtual bool  getIsOptimized() const { return (isOptimized_); }
    void setIsOptimized(bool b) { isOptimized_ = b; }
    friend std::ostream &operator<<(std::ostream &os, const Vertex &v);
  protected:
    // ----
    static std::string format_time(const ros::Time &t);
    std::string name_;
    std::string shape_;
    bool        isOptimized_{false};
  };
  typedef std::shared_ptr<Vertex> VertexPtr;
  std::ostream &operator<<(std::ostream &os, const Vertex &v);
}
