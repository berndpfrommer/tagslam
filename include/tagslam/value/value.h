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
      Value(const std::string &s = "", const ros::Time &t = ros::Time(0), bool valid = false):
        Vertex(s, "ellipse", t) {
        setIsValid(valid);
      }
      virtual ~Value() {}
      bool isValue() const override { return (true); }
      bool isOptimized() const override { return (key_ != 0); }
      virtual std::string getLabel() const override {
        return (Vertex::getLabel());
      }
      ValueKey getKey()  const { return (key_); }
      void     setKey(ValueKey k) { key_ = k; }
    private:
      ValueKey key_{0}; // zero is invalid!
    };
  }
  typedef std::shared_ptr<value::Value> ValuePtr;
  typedef std::shared_ptr<const value::Value> ValueConstPtr;

}
