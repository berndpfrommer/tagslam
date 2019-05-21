/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/boost_graph.h"
#include "tagslam/vertex_desc.h"
#include "tagslam/optimizer_key.h"
#include <string>
#include <iostream>
#include <memory>
#include <ros/ros.h>

namespace tagslam {
  // need forward declaration here
  typedef std::string VertexId;
  class Graph;
  class Vertex {
  public:
    typedef std::shared_ptr<Vertex> VertexPtr;
    typedef std::shared_ptr<const Vertex> VertexConstPtr;
    Vertex(const std::string &n = "", const std::string &s = "",
           const ros::Time &t= ros::Time(0)):
      name_(n), shape_(s), time_(t) {}
    virtual ~Vertex() {};
    virtual bool isValue() const = 0;
    virtual std::shared_ptr<Vertex> clone() const = 0;
    virtual VertexId   getId() const = 0;
    virtual VertexDesc addToGraph(const VertexPtr &vp, Graph *g) const = 0;

    virtual std::string getLabel() const { return (name_); }
    virtual std::string getShape() const { return (shape_); }
    virtual std::string toString() const;
    
    const std::string &getName() const { return (name_); }
    
    const ros::Time &getTime() const { return (time_); }
    friend std::ostream &operator<<(std::ostream &os, const Vertex &v);
    inline static VertexId make_id(const ros::Time &t,
                                   const std::string &name) {
      return (name + "_" + std::to_string(t.toNSec()));
    }
  protected:
    void checkIfValid(const VertexDesc &v, const std::string &m) const;
    // ----
    static std::string format_time(const ros::Time &t);
    std::string name_;
    std::string shape_;
    ros::Time   time_;
  };
  typedef Vertex::VertexPtr VertexPtr;
  typedef Vertex::VertexConstPtr VertexConstPtr;
  std::ostream &operator<<(std::ostream &os, const Vertex &v);
}
