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
      
      virtual std::vector<FactorKey> getKeys()  const = 0;
      virtual void clearKeys() = 0;
      bool isValue() const override { return (false); }

      static inline bool is_valid(FactorKey k) { return (k != Invalid); }
    protected:
      static const FactorKey Invalid{ULONG_MAX};
    };
  }
  typedef std::shared_ptr<factor::Factor> FactorPtr;
  typedef std::shared_ptr<const factor::Factor> FactorConstPtr;
}
