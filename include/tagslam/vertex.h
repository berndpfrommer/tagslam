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
    Vertex(const std::string &n = "", const std::string &s = "", const ros::Time &t= ros::Time(0)):
      name_(n), shape_(s), time_(t) {}
    virtual ~Vertex() {};
    virtual bool isValue() const = 0;
    virtual bool isOptimized() const = 0;
    virtual std::shared_ptr<Vertex> clone() const = 0;
    virtual VertexId   getId() const = 0;
    virtual VertexDesc attachTo(Graph *g) const = 0;
    virtual void addToOptimizer(Graph *g) = 0;

    virtual std::string getLabel() const { return (name_); }
    virtual std::string getShape() const { return (shape_); }
    virtual std::string toString() const;
    
    bool        isValid() const { return (isValid_); }
    void        setIsValid(bool v) { isValid_ = v; }
    const std::string &getName() const { return (name_); }
    
    const ros::Time &getTime() const { return (time_); }
    friend std::ostream &operator<<(std::ostream &os, const Vertex &v);
    inline static VertexId make_id(const ros::Time &t, const std::string &name) {
      return (name + "_" + std::to_string(t.toNSec()));
    }
  protected:
    // ----
    static std::string format_time(const ros::Time &t);
    std::string name_;
    std::string shape_;
    ros::Time   time_;
    bool        isValid_{false};
  };
  typedef std::shared_ptr<Vertex> VertexPtr;
  typedef std::shared_ptr<const Vertex> VertexConstPtr;
  std::ostream &operator<<(std::ostream &os, const Vertex &v);
}
