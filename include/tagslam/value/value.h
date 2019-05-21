/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/vertex.h"
#include "tagslam/value_key.h"

namespace tagslam {
  namespace value {
    class Value: public Vertex {
    public:
      Value(const std::string &s = "", const ros::Time &t = ros::Time(0)):
        Vertex(s, "ellipse", t) {
      }
      virtual ~Value() {}
      bool isValue() const override { return (true); }
      virtual std::string getLabel() const override {
        return (Vertex::getLabel());
      }
    };
  }
  typedef std::shared_ptr<value::Value> ValuePtr;
  typedef std::shared_ptr<const value::Value> ValueConstPtr;

}
