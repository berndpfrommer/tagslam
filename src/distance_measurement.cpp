/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/distance_measurement.h"
#include "tagslam/yaml_utils.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>

namespace tagslam {
  using boost::irange;
  DistanceMeasurementPtr
  DistanceMeasurement::parse(const std::string &name,
                             XmlRpc::XmlRpcValue meas) {
    DistanceMeasurementPtr m(new DistanceMeasurement(name));
    try {
      // first read the header that may contain the body pose
      for (XmlRpc::XmlRpcValue::iterator it = meas.begin();
           it != meas.end(); ++it) {
        if (it->first == "tag1") {
          m->tag1 = static_cast<int>(it->second);
        }
        if (it->first == "tag2") {
          m->tag2 = static_cast<int>(it->second);
        }
        if (it->first == "corner1") {
          m->corner1 = static_cast<int>(it->second);
        }
        if (it->first == "corner2") {
          m->corner2 = static_cast<int>(it->second);
        }
        if (it->first == "distance") {
          m->distance = static_cast<double>(it->second);
        }
        if (it->first == "noise") {
          m->noise = static_cast<double>(it->second);
        }
      }
    } catch (const XmlRpc::XmlRpcException &e) {
      throw std::runtime_error("error parsing measurement:" + name);
    }
    return (m);
  }

  DistanceMeasurementVec
  DistanceMeasurement::parse(XmlRpc::XmlRpcValue meas) {
    DistanceMeasurementVec mv;
    if (meas.getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
      throw std::runtime_error("invalid node type for measurement!");
    }
    for (const auto i: irange(0, meas.size())) {
      if (meas[i].getType() !=
          XmlRpc::XmlRpcValue::TypeStruct) continue;
      for (XmlRpc::XmlRpcValue::iterator it = meas[i].begin();
           it != meas[i].end(); ++it) {
        if (it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
          continue;
        }
        DistanceMeasurementPtr m = parse(it->first, it->second);
        mv.push_back(m);
      }
    }
    return (mv);
  }

}  // namespace
