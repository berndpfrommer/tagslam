/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/measurements/measurements.h"
#include "tagslam/measurements/distance.h"
#include <XmlRpcException.h>

#include <boost/range/irange.hpp>

namespace tagslam {
  namespace measurements {
    using boost::irange;
    std::vector<MeasurementsPtr>
    read_all(XmlRpc::XmlRpcValue config,
             const GraphPtr &graph, TagFactory *tagFactory) {
      std::vector<MeasurementsPtr> meas;
      MeasurementsPtr m =
        measurements::Distance::read(config, graph, tagFactory);
      if (m) { meas.push_back(m); }
      return (meas);
    }
  } // end of namespace
}  // end of namespace

