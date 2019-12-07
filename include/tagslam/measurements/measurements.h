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

      virtual void addToGraph(const GraphPtr &graph);
      virtual void tryAddToOptimizer(const GraphPtr &graph);
      virtual void printUnused(const GraphConstPtr &graph);
      
      virtual void writeDiagnostics(const GraphPtr &graph) = 0;

    protected:
      template <typename T>
      static std::shared_ptr<T>cast(const FactorPtr &factor) {
        std::shared_ptr<T> fp =
          std::dynamic_pointer_cast<T>(factor);
        if (!fp) {
          ROS_ERROR_STREAM("wrong factor type: " << *factor);
          throw (std::runtime_error("wrong factor type"));
        }
        return (fp);
      }
      std::vector<VertexDesc> vertexes_;
      std::vector<FactorPtr>  factors_;
    };
    typedef Measurements::MeasurementsPtr MeasurementsPtr;
    typedef Measurements::MeasurementsConstPtr MeasurementsConstPtr;
    std::vector<MeasurementsPtr>
    read_all(XmlRpc::XmlRpcValue config, TagFactory *tagFactory);
  }
  typedef measurements::MeasurementsPtr MeasurementsPtr;
  typedef measurements::MeasurementsConstPtr MeasurementsConstPtr;
}
