/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/measurements/distance.h"
#include "tagslam/body.h"
#include "tagslam/logging.h"

#include <XmlRpcException.h>

#include <fstream>
#include <iomanip>

namespace tagslam {
  namespace measurements {

#define FMT(X, Y) std::fixed << std::setw(X) << std::setprecision(Y)

    void Distance::writeDiagnostics() {
      std::ofstream f("distance_diagnostics.txt");
      for (const auto &v: vertexes_) {
        const auto p = factor::Distance::cast_const((*graph_)[v]);
        if (graph_->isOptimized(v)) {
          const double l = factor::Distance::getOptimized(v, *graph_);
          const double diff = l - p->getDistance();
          f << FMT(6, 3) << graph_->getError(v)
            << " diff: " << FMT(6, 3) << diff
            << " opt: "  << FMT(6, 3) << l << " meas: "
            << FMT(6, 3) << p->getDistance() << " " << *p << std::endl;
        }
      }
    }

    Distance::DistanceMeasurementsPtr
    Distance::read(XmlRpc::XmlRpcValue config, TagFactory *tagFactory) {
      if (!config.hasMember("distance_measurements")) {
        ROS_INFO_STREAM("no distance measurements found!");
        return DistanceMeasurementsPtr();
      }
      std::shared_ptr<measurements::Distance>
        m(new measurements::Distance());
      XmlRpc::XmlRpcValue meas = config[std::string("distance_measurements")];
      if (meas.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        auto factors = factor::Distance::parse(meas, tagFactory);
        for (const auto f: factors) {
          if (!f->getTag(0)->getBody()->isStatic()
              || !f->getTag(1)->getBody()->isStatic()) {
            BOMB_OUT("measured bodies must be static: " << *f);
          }
          ROS_INFO_STREAM("found distance: " << *f);
          m->factors_.push_back(f);
        }
      } else {
        ROS_INFO_STREAM("no distance measurements found!");
      }
      return (m);
    }
  }  // end of namespace
}  // end of namespace


