/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/factor/factor.h"
#include "tagslam/tag_factory.h"
#include "tagslam/geometry.h"
#include "tagslam/tag.h"
#include <ros/ros.h>

#include <string>

namespace tagslam {
  namespace factor {
    using std::string;
    class Distance: public Factor {
    public:
      typedef std::shared_ptr<factor::Distance> DistanceFactorPtr;
      typedef std::shared_ptr<const factor::Distance> DistanceFactorConstPtr;
      typedef std::vector<DistanceFactorPtr> DistanceFactorPtrVec;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      Distance(double dist,  double noise,
               const int corn1, const TagConstPtr &tag1,
               const int corn2, const TagConstPtr &tag2,
               const string  &name);
      // ------ inherited methods -----
      string getLabel() const override;
      VertexId    getId() const override { return (name_);}
      std::shared_ptr<Vertex> clone() const override {
        return (std::shared_ptr<Distance>(new Distance(*this))); }
      VertexDesc addToGraph(const VertexPtr &vp, Graph *g) const override;

      void addToOptimizer(Graph *g) const override;
      bool establishesValues() const override { return (false); }

      // --------- own methods
      double getDistance() const { return (distance_); }
      double getNoise()    const { return (noise_); }
      double distance(const Transform &T_w_b1,
                      const Transform &T_b1_o,
                      const Transform &T_w_b2,
                      const Transform &T_b2_o) const;
      const TagConstPtr getTag(int idx) const { return (tag_[idx]); }
      const Eigen::Vector3d getCorner(int idx) const;
      // --- static methods
      inline static std::shared_ptr<const factor::Distance> cast_const(
        const VertexPtr &vp) {
        return (std::dynamic_pointer_cast<const factor::Distance>(vp));
      }
      inline static std::shared_ptr<factor::Distance> cast(
        const VertexPtr &vp) {
        return (std::dynamic_pointer_cast<factor::Distance>(vp));
      }
      static DistanceFactorPtrVec parse(XmlRpc::XmlRpcValue meas,
                                        TagFactory *tf);
      static double getOptimized(const VertexDesc &v, const Graph &g);

    private:
      static DistanceFactorPtr parse(const string &name,
                                     XmlRpc::XmlRpcValue meas,
                                     TagFactory *factory);
      double       distance_;
      double       noise_;
      TagConstPtr  tag_[2];
      int          corner_[2];
    };
  }
  typedef factor::Distance::DistanceFactorPtr      DistanceFactorPtr;
  typedef factor::Distance::DistanceFactorConstPtr DistanceFactorConstPtr;
  typedef factor::Distance::DistanceFactorPtrVec   DistanceFactorPtrVec;
}
