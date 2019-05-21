/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/measurements/measurements.h"
#include "tagslam/geometry.h"
#include "tagslam/vertex.h"
#include <vector>

namespace tagslam {
  namespace measurements {
    class Plane: public Measurements {
    public:
      typedef std::shared_ptr<Plane> PlaneMeasurementsPtr;
      
      VertexDesc addFactorToGraph(const FactorPtr &factor) override {
        return (factor->addToGraph(factor, graph_.get()));
      }
      void addFactorToOptimizer(const FactorPtr &factor) override {
        factor->addToOptimizer(graph_.get());
      }

      void writeDiagnostics() override;

      // static functions
      static PlaneMeasurementsPtr read(XmlRpc::XmlRpcValue config,
                                       TagFactory *fac);
    private:
      // static functions
      static std::vector<FactorPtr> generate_factors(
        const std::string &name, double d, double noise,
        const Point3d &dir, const std::vector<int> &tags, TagFactory *tagFac);
    };
  }
}
