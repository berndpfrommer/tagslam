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
  class Vertex {
  public:
    Vertex(const std::string &n = "", const std::string &s = ""):
      name_(n), shape_(s) {}
    virtual ~Vertex() {};
    virtual bool isValue() const = 0;
    virtual bool isOptimized() const = 0;

    bool isValid() const { return (isValid_); }
    void setIsValid(bool v) { isValid_ = v; }
    
    virtual std::string getLabel() const { return (name_); }
    virtual std::string getShape() const { return (shape_); }
    friend std::ostream &operator<<(std::ostream &os, const Vertex &v);
  protected:
    // ----
    static std::string format_time(const ros::Time &t);
    std::string name_;
    std::string shape_;
    bool        isValid_{false};
  };
  typedef std::shared_ptr<Vertex> VertexPtr;
  typedef std::shared_ptr<const Vertex> VertexConstPtr;
  std::ostream &operator<<(std::ostream &os, const Vertex &v);
}
