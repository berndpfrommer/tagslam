/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <string>
#include <iostream>

namespace tagslam {
  class Vertex {
  public:
    Vertex(const std::string &n = "", const std::string &s = ""):
      name(n), shape(s) {}
    virtual ~Vertex() {};
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
