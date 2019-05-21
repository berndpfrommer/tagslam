/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/factor/factor.h"
#include "tagslam/tag_factory.h"
#include "tagslam/geometry.h"
#include "tagslam/tag.h"

#include <ros/ros.h>
#include <memory>

namespace tagslam {
  namespace factor {
    using std::string;
    class Coordinate: public Factor {
    public:
      typedef std::shared_ptr<factor::Coordinate> CoordinateFactorPtr;
      typedef std::shared_ptr<const factor::Coordinate>
      CoordinateFactorConstPtr;
      typedef std::vector<CoordinateFactorPtr> CoordinateFactorPtrVec;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      Coordinate(double len,  double noise, const Point3d &direction,
                 const int corn, const TagConstPtr &tag,
                 const string  &name);
      // ------ inherited methods -----
      string getLabel() const override;
      VertexId    getId() const override { return (name_);}
      std::shared_ptr<Vertex> clone() const override {
        return (std::shared_ptr<Coordinate>(new Coordinate(*this))); }
      VertexDesc addToGraph(const VertexPtr &vp, Graph *g) const override;
      void addToOptimizer(Graph *g) const override;
      bool establishesValues() const override { return (false); }

      // --------- own methods
      double getLength() const { return (length_); }
      double getNoise()    const { return (noise_); }
      const Eigen::Vector3d &getDirection() const { return (direction_); }
      double coordinate(const Transform &T_w_b,
                        const Transform &T_b_o) const;
      const TagConstPtr getTag() const { return (tag_); }
      const Eigen::Vector3d getCorner() const;

      // --- static methods
      inline static std::shared_ptr<const factor::Coordinate> cast_const(
        const VertexPtr &vp) {
        return (std::dynamic_pointer_cast<const factor::Coordinate>(vp));
      }
      inline static std::shared_ptr<factor::Coordinate> cast(
        const VertexPtr &vp) {
        return (std::dynamic_pointer_cast<factor::Coordinate>(vp));
      }
      static CoordinateFactorPtrVec parse(XmlRpc::XmlRpcValue meas,
                                          TagFactory *tf);
      static double getOptimized(const VertexDesc &v, const Graph &g);

    private:
      static CoordinateFactorPtr parse(const string &name,
                                       XmlRpc::XmlRpcValue meas,
                                       TagFactory *factory);
      double          length_;
      double          noise_;
      Eigen::Vector3d direction_;
      int             corner_;
      TagConstPtr     tag_;
    };
  }
  typedef factor::Coordinate::CoordinateFactorPtr      CoordinateFactorPtr;
  typedef factor::Coordinate::CoordinateFactorConstPtr CoordinateFactorConstPtr;
  typedef factor::Coordinate::CoordinateFactorPtrVec   CoordinateFactorPtrVec;
}
