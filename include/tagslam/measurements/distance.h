/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/measurements/measurements.h"
#include "tagslam/vertex.h"
#include <vector>

namespace tagslam {
  namespace measurements {
    class Distance: public Measurements {
    public:
      typedef std::shared_ptr<Distance> DistanceMeasurementsPtr;
      
      void apply() override;
      void printUnused() override;
      void writeDiagnostics() override;

      // static functions
      static DistanceMeasurementsPtr
      read(XmlRpc::XmlRpcValue config, const GraphPtr &graph,
           TagFactory *fac);
    private:
      GraphPtr                graph_;
      std::vector<VertexDesc> measurements_;
      std::vector<VertexDesc> unappliedMeasurements_;
    };
  }
}
