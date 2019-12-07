/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/measurements/measurements.h"
#include "tagslam/measurements/distance.h"
#include "tagslam/measurements/coordinate.h"
#include "tagslam/measurements/plane.h"
#include <XmlRpcException.h>

#include <boost/range/irange.hpp>

namespace tagslam {
  namespace measurements {
    using boost::irange;
    std::vector<MeasurementsPtr>
    read_all(XmlRpc::XmlRpcValue config, TagFactory *tagFactory) {
      std::vector<MeasurementsPtr> meas;
      MeasurementsPtr m;

      m = measurements::Distance::read(config, tagFactory);
      if (m) { meas.push_back(m); }
      
      m = measurements::Coordinate::read(config, tagFactory);
      if (m) { meas.push_back(m); }

      m = measurements::Plane::read(config, tagFactory);
      if (m) { meas.push_back(m); }
    
      return (meas);
    }

    void Measurements::addToGraph(const GraphPtr &graph) {
      for (const auto &f: factors_) {
        vertexes_.push_back(f->addToGraph(f, graph.get()));
      }
    }

    void Measurements::tryAddToOptimizer(const GraphPtr &graph) {
      for (const auto &v: vertexes_) {
        if (graph->isOptimizableFactor(v) && !graph->isOptimized(v)) {
          auto fp = std::dynamic_pointer_cast<factor::Factor>((*graph)[v]);
          fp->addToOptimizer(graph.get());
        }
      }
    }

    void Measurements::printUnused(const GraphConstPtr &graph) {
      for (const auto &f: vertexes_) {
        if (!graph->isOptimized(f)) {
          ROS_INFO_STREAM("unused measurement: " << (*graph)[f]);
        }
      }
    }


  } // end of namespace
}  // end of namespace

