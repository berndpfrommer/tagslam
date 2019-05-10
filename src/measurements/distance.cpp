/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/measurements/distance.h"
#include "tagslam/body.h"
#include <XmlRpcException.h>

#include <boost/range/irange.hpp>
#include <fstream>
#include <iomanip>

namespace tagslam {
  namespace measurements {
    void Distance::apply() {
      if (unappliedMeasurements_.empty()) {
        return;
      }
      std::vector<VertexDesc> remaining;
      for (const auto &v: unappliedMeasurements_) {
        if (graph_->isOptimizableFactor(v)) {
          DistanceFactorConstPtr fp =
            std::dynamic_pointer_cast<const factor::Distance>((*graph_)[v]);
          if (fp) {
            graph_->addToOptimizer(fp.get());
          } else {
            ROS_ERROR_STREAM("wrong factor type: " << graph_->info(v));
            throw (std::runtime_error("wrong factor type"));
          }
        } else {
          remaining.push_back(v);
        }
      }
      unappliedMeasurements_ = remaining;
    }

    void Distance::printUnused() {
      for (const auto &v: unappliedMeasurements_) {
        ROS_INFO_STREAM("unapplied factor: " << graph_->info(v));
      }
    }
    
#define FMT(X, Y) std::fixed << std::setw(X) << std::setprecision(Y)

    void Distance::writeDiagnostics() {
      std::ofstream f("distance_diagnostics.txt");
      for (const auto &v: measurements_) {
        const auto p = factor::Distance::cast_const((*graph_)[v]);
        const double l = graph_->getOptimizedDistance(v);
        const double diff = l - p->getDistance();
        f << FMT(6, 3) << graph_->getError(v)  << " diff: " << FMT(6, 3) << diff
          << " opt: " << FMT(6, 3) << l << " meas: "
          << FMT(6, 3) << p->getDistance() << " " << *p << std::endl;
      }
    }

    Distance::DistanceMeasurementsPtr
    Distance::read(XmlRpc::XmlRpcValue config,
                   const GraphPtr &graph, TagFactory *tagFactory) {
      if (!config.hasMember("distance_measurements")) {
        ROS_INFO_STREAM("no distance measurements found!");
        return DistanceMeasurementsPtr();
      }
      std::shared_ptr<measurements::Distance>
        m(new measurements::Distance());
      m->graph_ = graph;
      XmlRpc::XmlRpcValue meas = config[std::string("distance_measurements")];
      if (meas.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        auto factors = factor::Distance::parse(meas, tagFactory);
        for (const auto f: factors) {
          if (!f->getTag(0)->getBody()->isStatic()
              || !f->getTag(1)->getBody()->isStatic()) {
            ROS_ERROR_STREAM("measured bodies must be static: " << *f);
            throw (std::runtime_error("measured bodies must be static!"));
          }
          ROS_INFO_STREAM("found distance: " << *f);
          m->measurements_.push_back(graph->add(f));
        }
      } else {
        ROS_INFO_STREAM("no distance measurements found!");
      }
      m->unappliedMeasurements_ = m->measurements_;
      return (m);
    }
  }  // end of namespace
}  // end of namespace


