/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/vertex.h"
#include "tagslam/factor_key.h"
#include <ros/ros.h>
#include <string>

namespace tagslam {
  namespace factor {
    using std::string;
    class Factor: public Vertex {
    public:
      Factor(const string &s = "", const ros::Time &t = ros::Time(0)):
        Vertex(s, "box", t) {}
      virtual ~Factor() {}
      virtual void addToOptimizer(Graph *g) const = 0;
      virtual bool establishesValues() const = 0;
      bool isValue() const override { return (false); }
    protected:
    };
    template <typename T>
    static bool is(const VertexConstPtr &vp) {
      return (std::dynamic_pointer_cast<const T>(vp) != 0);
    }
    inline static std::shared_ptr<const factor::Factor> cast_const(
      const VertexPtr &vp) {
      return (std::dynamic_pointer_cast<const factor::Factor>(vp));
    }
    inline static std::shared_ptr<factor::Factor> cast(
      const VertexPtr &vp) {
      return (std::dynamic_pointer_cast<factor::Factor>(vp));
    }
  }
  typedef std::shared_ptr<factor::Factor> FactorPtr;
  typedef std::shared_ptr<const factor::Factor> FactorConstPtr;
}
