/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/measurements/plane.h"
#include "tagslam/body.h"
#include "tagslam/xml.h"
#include "tagslam/logging.h"
#include <XmlRpcException.h>

#include <boost/range/irange.hpp>
#include <fstream>
#include <iomanip>

namespace tagslam {
  using boost::irange;
  namespace measurements {
    
#define FMT(X, Y) std::fixed << std::setw(X) << std::setprecision(Y)

    void Plane::writeDiagnostics() {
      std::ofstream f("plane_diagnostics.txt");
      for (const auto &v: vertexes_) {
        if (graph_->isOptimized(v)) {
          const auto p = factor::Coordinate::cast_const((*graph_)[v]);
          const double l = factor::Coordinate::getOptimized(v, *graph_);
          const double diff = l - p->getLength();
          f << FMT(6, 3) << graph_->getError(v)
            << " diff: " << FMT(6, 3) << diff
            << " opt: " << FMT(6, 3) << l << " meas: "
            << FMT(6, 3) << p->getLength() << " " << *p << std::endl;
        }
      }
    }
    std::vector<FactorPtr> Plane::generate_factors(
      const std::string &name, double d, double noise,
      const Point3d &dir, const std::vector<int> &tags, TagFactory *tagFac) {
      std::vector<FactorPtr> factors;
      for (const auto &tag: tags) {
        std::string coordName = name + "_tag_" + std::to_string(tag);
        TagConstPtr tagPtr = tagFac->findTag(tag);
        if (!tagPtr) {
          BOMB_OUT("measured tag is not valid: " << tag);
        }
        CoordinateFactorPtr fp(
          new factor::Coordinate(d, noise, dir, -1, tagPtr, coordName));
        factors.push_back(fp);
      }
      return (factors);
    }

    Plane::PlaneMeasurementsPtr
    Plane::read(XmlRpc::XmlRpcValue config, TagFactory *tagFactory) {
      if (!config.hasMember("plane_measurements")) {
        ROS_INFO_STREAM("no plane measurements found!");
        return PlaneMeasurementsPtr();
      }
      const std::shared_ptr<measurements::Plane> m(new measurements::Plane());
      XmlRpc::XmlRpcValue meas = config[std::string("plane_measurements")];
      if (meas.getType() != XmlRpc::XmlRpcValue::TypeArray
          || meas.size() == 0) {
        BOMB_OUT("plane measurements must be valid array!");
      }
      ROS_INFO_STREAM("found " << meas.size() << " plane measurement(s)!");
      for (const auto i: irange(0, meas.size())) { // iterate over planes
        try {
          if (meas[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            BOMB_OUT("plane " << i << " is not struct!");
          }
          if (meas[i].size() != 1) {
            BOMB_OUT("plane " << i << " has wrong number of fields");
          }
          const std::string name = meas[i].begin()->first;
          XmlRpc::XmlRpcValue &plane = meas[i].begin()->second;
          const std::vector<int> tags =
            xml::parse_container<std::vector<int>>(plane, "tags");
          const double d     = xml::parse<double>(plane, "distance");
          const double noise = xml::parse<double>(plane, "noise");
          const Point3d dir = make_point(
            xml::parse_container<std::vector<double>>(plane, "direction"));
          if (std::abs(dir.norm() - 1.0) > 1e-5) {
            BOMB_OUT("plane has non-unit direction");
          }
          ROS_INFO_STREAM("found plane measurement: " << name);
          m->factors_ = generate_factors(name, d, noise, dir, tags,tagFactory);
        } catch (const XmlRpc::XmlRpcException &e) {
          BOMB_OUT("config error with plane " << meas[i].begin()->first);
        }
      }
      return (m);
    }
  }  // end of namespace
}  // end of namespace


