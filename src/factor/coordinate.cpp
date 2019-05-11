/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/factor/coordinate.h"
#include "tagslam/graph.h"
#include "tagslam/body.h"
#include <geometry_msgs/Point.h>
#include <XmlRpcException.h>
#include <string>
#include <sstream>
#include <boost/range/irange.hpp>

namespace tagslam {
  namespace factor {
    using boost::irange;
    
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
      g->addToOptimizer(this);
    }

    Coordinate::Coordinate(double len,  double noise,
                           const Eigen::Vector3d direction,
                           const int corn, const Tag2ConstPtr &tag,
                           const std::string  &name) :
      Factor(name, ros::Time(0)),
      length_(len), noise_(noise),
      direction_(direction), corner_(corn),
      tag_(tag) {
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
    Coordinate::parse(const std::string &name, XmlRpc::XmlRpcValue meas,
                    TagFactory *tagFactory) {
      int tag(-1), c(-1);
      double len(-1e10), noise(-1);
      Eigen::Vector3d dir(0.0, 0.0, 0.0);
      try {
        for (XmlRpc::XmlRpcValue::iterator it = meas.begin();
             it != meas.end(); ++it) {
          if (it->first == "tag") { tag = static_cast<int>(it->second); }
          if (it->first == "corner") { c = static_cast<int>(it->second); }
          if (it->first == "length") { len = static_cast<double>(it->second); }
          if (it->first == "noise") { noise = static_cast<double>(it->second);}
          if (it->first == "direction") {
            for (int j = 0; j < std::min(it->second.size(), 3); j++) {
              if (it->second[j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
                throw (std::runtime_error("bad value for direction " + name));
              } else {
                dir(j) = static_cast<double>(it->second[j]);
              }
            }
          }
        }
      } catch (const XmlRpc::XmlRpcException &e) {
        ROS_ERROR_STREAM("error parsing measurement: " << name);
        throw std::runtime_error("error parsing measurement:" + name);
      }
      if (std::abs(dir.norm() - 1.0) > 1e-5) {
        ROS_ERROR_STREAM("measurement " << name << " has non-unit direction");
        throw std::runtime_error("non-unit direction for meas " + name);
      }
      CoordinateFactorPtr fp;
      if (tag >= 0 && c >= 0 && len > -1e10 && noise > 0) {
        Tag2ConstPtr tagPtr = tagFactory->findTag(tag);
        if (!tagPtr) {
          ROS_ERROR_STREAM("measured tag is not valid: " << name);
          throw (std::runtime_error("measured tag is not valid!"));
        }
        fp.reset(new factor::Coordinate(len, noise, dir, c, tagPtr,
                                        name));
      } else {                                  
        ROS_ERROR_STREAM("coordinate measurement incomplete: " << name);
        throw (std::runtime_error("coordinate measurement incomplete!"));
      }
      return (fp);
    }

    CoordinateFactorPtrVec
    Coordinate::parse(XmlRpc::XmlRpcValue meas, TagFactory *tagFactory) {
      CoordinateFactorPtrVec fv;
      if (meas.getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
        throw std::runtime_error("invalid node type for measurement!");
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
    
    std::string Coordinate::getLabel() const {
      std::stringstream ss;
      ss << name_;
      return (ss.str());
    }
  } // namespace factor
}  // namespace tagslam
