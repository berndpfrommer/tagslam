/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/measurements/measurements.h"
#include "tagslam/factor/distance.h"
#include "tagslam/vertex.h"
#include <vector>

namespace tagslam {
  namespace measurements {
    class Distance: public Measurements {
    public:
      typedef std::shared_ptr<Distance> DistanceMeasurementsPtr;

      void writeDiagnostics(const GraphPtr &graph) override;

      // static functions
      static DistanceMeasurementsPtr read(XmlRpc::XmlRpcValue config,
                                          TagFactory *fac);
    private:
    };
  }
}
