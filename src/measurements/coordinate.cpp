/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/measurements/coordinate.h"
#include "tagslam/body.h"
#include "tagslam/logging.h"

#include <XmlRpcException.h>

#include <fstream>
#include <iomanip>

namespace tagslam {
  namespace measurements {
    
#define FMT(X, Y) std::fixed << std::setw(X) << std::setprecision(Y)

    void Coordinate::writeDiagnostics() {
      std::ofstream f("coordinate_diagnostics.txt");
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

    Coordinate::CoordinateMeasurementsPtr
    Coordinate::read(XmlRpc::XmlRpcValue config,
                     TagFactory *tagFactory) {
      if (!config.hasMember("coordinate_measurements")) {
        ROS_INFO_STREAM("no coordinate measurements found!");
        return CoordinateMeasurementsPtr();
      }
      std::shared_ptr<measurements::Coordinate>
        m(new measurements::Coordinate());
      XmlRpc::XmlRpcValue meas =
        config[std::string("coordinate_measurements")];
      if (meas.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        auto fpts = factor::Coordinate::parse(meas, tagFactory);
        for (const auto f: fpts) {
          if (!f->getTag()->getBody()->isStatic()) {
            BOMB_OUT("measured body must be static: " << *f);
          }
          ROS_INFO_STREAM("found coordinate: " << *f);
          m->factors_.push_back(f);
        }
      } else {
        ROS_INFO_STREAM("no coordinate measurements found!");
      }
      return (m);
    }
  }  // end of namespace
}  // end of namespace


