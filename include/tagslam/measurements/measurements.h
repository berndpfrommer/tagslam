/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <vector>
#include <memory>
#include <ros/ros.h>
#include "tagslam/tag_factory.h"
#include "tagslam/graph.h"

namespace tagslam {
  namespace measurements {
    class Measurements {
    public:
      virtual ~Measurements() {}
      typedef std::shared_ptr<Measurements> MeasurementsPtr;
      typedef std::shared_ptr<const Measurements> MeasurementsConstPtr;
      virtual void apply() = 0;
      virtual void printUnused() = 0;
      virtual void writeDiagnostics() = 0;
 
    };
    typedef Measurements::MeasurementsPtr MeasurementsPtr;
    typedef Measurements::MeasurementsConstPtr MeasurementsConstPtr;
    std::vector<MeasurementsPtr>
    read_all(XmlRpc::XmlRpcValue config,
             const GraphPtr &graph, TagFactory *tagFactory);
  }
  typedef measurements::MeasurementsPtr MeasurementsPtr;
  typedef measurements::MeasurementsConstPtr MeasurementsConstPtr;
}
