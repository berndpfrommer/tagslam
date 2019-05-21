/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/factor/coordinate.h"
#include "tagslam/graph.h"
#include "tagslam/body.h"
#include "tagslam/logging.h"
#include "tagslam/xml.h"
#include <geometry_msgs/Point.h>
#include <XmlRpcException.h>
#include <string>
#include <sstream>
#include <boost/range/irange.hpp>

namespace tagslam {
  namespace factor {
    using boost::irange;
    
    Coordinate::Coordinate(double len, double noise, const Point3d &direction,
                           const int corn, const TagConstPtr &tag,
                           const string &name) :
      Factor(name, ros::Time(0)),
      length_(len), noise_(noise),
      direction_(direction), corner_(corn),
      tag_(tag) {
    }
    
    VertexDesc Coordinate::addToGraph(const VertexPtr &vp, Graph *g) const {
      const ros::Time t0 = ros::Time(0);
      const VertexDesc vtp = g->findTagPose(getTag()->getId());
      checkIfValid(vtp, "no tag pose found");
      const VertexDesc vbp = g->findBodyPose(t0,
                                             getTag()->getBody()->getName());
      checkIfValid(vbp, "no body pose found");
      const VertexDesc fv = g->insertFactor(vp);
      g->addEdge(fv, vbp, 0);
      g->addEdge(fv, vtp, 1);
      return (fv);
    }

    void Coordinate::addToOptimizer(Graph *g) const {
      const VertexDesc v = g->find(this);
      checkIfValid(v, "factor not found");
      const std::vector<ValueKey> optKeys = g->getOptKeysForFactor(v, 2);
      const FactorKey fk = g->getOptimizer()->addCoordinateMeasurement(
        getLength(), getNoise(),
        getDirection(), getCorner(),
        optKeys[0] /* T_w_b */, optKeys[1] /* T_b_o */);
      g->markAsOptimized(v, fk);
    }

    double Coordinate::coordinate(
      const Transform &T_w_b, const Transform &T_b_o) const {
      const auto X = T_w_b * T_b_o * getCorner();
      return (direction_.dot(X));
    }

    const Eigen::Vector3d Coordinate::getCorner() const {
      return (tag_->getObjectCorner(corner_));
    }

    CoordinateFactorPtr
    Coordinate::parse(const string &name, XmlRpc::XmlRpcValue meas,
                    TagFactory *tagFactory) {
      int tag(-1), c(-1);
      double len(-1e10), noise(-1);
      Eigen::Vector3d dir(0.0, 0.0, 0.0);
      try {
        tag   = xml::parse<int>(meas, "tag");
        c     = xml::parse<int>(meas, "corner");
        len   = xml::parse<double>(meas, "length");
        noise = xml::parse<double>(meas, "noise");
        dir   = make_point(
          xml::parse_container<std::vector<double>>(meas, "direction"));
      } catch (const XmlRpc::XmlRpcException &e) {
        BOMB_OUT("error parsing measurement: " << name);
      }
      if (std::abs(dir.norm() - 1.0) > 1e-5) {
        BOMB_OUT("measurement " + name + " has non-unit direction");
      }
      CoordinateFactorPtr fp;
      if (tag >= 0 && c >= 0 && len > -1e10 && noise > 0) {
        TagConstPtr tagPtr = tagFactory->findTag(tag);
        if (!tagPtr) {
          BOMB_OUT("measured tag is not valid: " << name);
        }
        fp.reset(new factor::Coordinate(len, noise, dir, c, tagPtr,
                                        name));
      } else {                                  
        BOMB_OUT("coordinate measurement incomplete: " << name);
      }
      return (fp);
    }

    CoordinateFactorPtrVec
    Coordinate::parse(XmlRpc::XmlRpcValue meas, TagFactory *tagFactory) {
      CoordinateFactorPtrVec fv;
      if (meas.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        BOMB_OUT("invalid node type for coordinate measurements!");
      }
      for (const auto i: irange(0, meas.size())) {
        if (meas[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;
        for (XmlRpc::XmlRpcValue::iterator it = meas[i].begin();
             it != meas[i].end(); ++it) {
          if (it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            continue;
          }
          CoordinateFactorPtr d = parse(it->first, it->second, tagFactory);
          if (d) {
            fv.push_back(d);
          }
        }
      }
      return (fv);
    }
    
    string Coordinate::getLabel() const {
      std::stringstream ss;
      ss << name_;
      return (ss.str());
    }
    // static function!
    double Coordinate::getOptimized(const VertexDesc &v, const Graph &g) {
      if (!g.isOptimized(v)) {
        return (-1.0); // not optimized yet!
      }
      const auto p = std::dynamic_pointer_cast<const factor::Coordinate>(g[v]);
      if (!p) {
        BOMB_OUT("vertex is not coord: " << *g[v]);
      }
      const std::vector<ValueKey> optKeys = g.getOptKeysForFactor(v, 2);
      const auto opt = g.getOptimizer();
      const double l = p->coordinate(opt->getPose(optKeys[0]),
                                     opt->getPose(optKeys[1]));
      return (l);
    }
  } // namespace factor
}  // namespace tagslam
