/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/vertex.h"
#include "tagslam/factor_key.h"
#include <ros/ros.h>

namespace tagslam {
  namespace factor {
    class Factor: public Vertex {
    public:
      Factor(const std::string &s = "", const ros::Time &t = ros::Time(0)):
        Vertex(s, "box", t) {}
      virtual ~Factor() {}
      virtual void addToOptimizer(Graph *g) const = 0;
      bool isValue() const override { return (false); }
    protected:
    };
  }
  typedef std::shared_ptr<factor::Factor> FactorPtr;
  typedef std::shared_ptr<const factor::Factor> FactorConstPtr;
}
