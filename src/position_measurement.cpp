/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/position_measurement.h"
#include "tagslam/yaml_utils.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>

namespace tagslam {
  using boost::irange;
  PositionMeasurementPtr
  PositionMeasurement::parse(const std::string &name,
                             XmlRpc::XmlRpcValue meas) {
    PositionMeasurementPtr m(new PositionMeasurement(name));
    try {
      // first read the header that may contain the body pose
      for (XmlRpc::XmlRpcValue::iterator it = meas.begin();
           it != meas.end(); ++it) {
        if (it->first == "tag") {
          m->tag = static_cast<int>(it->second);
        }
        if (it->first == "corner") {
          m->corner = static_cast<int>(it->second);
        }

        if (it->first == "direction") {
          double v[3] = {0, 0, 0};
          for (int j = 0; j < std::min(it->second.size(), 3); j++) {
            if (it->second[j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
              throw (std::runtime_error("bad value for direction " + name));
            } else {
              v[j] = static_cast<double>(it->second[j]);
            }
          }
          m->dir = gtsam::Point3(v[0], v[1], v[2]);
        }
        if (it->first == "length") {
          m->length = static_cast<double>(it->second);
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

  PositionMeasurementVec
  PositionMeasurement::parse(XmlRpc::XmlRpcValue meas) {
    PositionMeasurementVec mv;
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
        PositionMeasurementPtr m = parse(it->first, it->second);
        mv.push_back(m);
      }
    }
    return (mv);
  }

}  // namespace
