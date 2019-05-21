/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/xml.h"
namespace tagslam {
  namespace xml {
    template <>
    Point3d parse(XmlRpc::XmlRpcValue xml, const std::string &key) {
      if (!xml.hasMember(key)) {
        ROS_ERROR_STREAM("key not found: " << key);
        throw XmlRpc::XmlRpcException("key not found: " + key);
      }
      try {
        const Point3d v(parse<double>(xml[key], "x"),
                        parse<double>(xml[key], "y"),
                        parse<double>(xml[key], "z"));
        return (v);
      } catch (const XmlRpc::XmlRpcException &e) {
        ROS_ERROR_STREAM("error parsing: " << key);
        throw (e);
      }
    }

    template <>
    Transform parse(XmlRpc::XmlRpcValue xml, const std::string &key) {
      if (!xml.hasMember(key)) {
        ROS_ERROR_STREAM("key not found: " << key);
        throw XmlRpc::XmlRpcException("key not found: " + key);
      }
      try {
        const Point3d p = parse<Point3d>(xml[key], "position");
        const Point3d r = parse<Point3d>(xml[key], "rotation");
        return (make_transform(r, p));
      } catch (const XmlRpc::XmlRpcException &e) {
        ROS_ERROR_STREAM("error parsing: " << key);
        throw (e);
      }
    }

    template <>
    PoseNoise2 parse(XmlRpc::XmlRpcValue xml, const std::string &key) {
      if (!xml.hasMember(key)) {
        ROS_ERROR_STREAM("key not found: " << key);
        throw XmlRpc::XmlRpcException("key not found: " + key);
      }
      try {
        if (xml[key].hasMember("position_noise") &&
            xml[key].hasMember("rotation_noise")) {
          const Point3d p = parse<Point3d>(xml[key], "position_noise");
          const Point3d r = parse<Point3d>(xml[key], "rotation_noise");
          return (PoseNoise2::make(r, p));
        } else if (xml[key].hasMember("R")) {
          auto Rd = parse_container<std::vector<double>>(xml[key], "R");
          const auto R = Eigen::Map<Eigen::Matrix<double, 6, 6> >(&Rd[0]);
          return (PoseNoise2::makeFromR(R));
        } else {
          throw XmlRpc::XmlRpcException("no valid noise for: " + key);
        }
      } catch (const XmlRpc::XmlRpcException &e) {
        ROS_ERROR_STREAM("error parsing: " << key);
        throw (e);
      }
    }
    
    template <>
    PoseWithNoise parse(XmlRpc::XmlRpcValue xml, const std::string &key) {
      if (!xml.hasMember(key)) {
        ROS_ERROR_STREAM("key not found: " << key);
        throw XmlRpc::XmlRpcException("key not found: " + key);
      }
      try {
        const Transform  pose  = parse<Transform>(xml, key);
        const PoseNoise2 noise = parse<PoseNoise2>(xml, key);
        return (PoseWithNoise(pose, noise, true));
      } catch (const XmlRpc::XmlRpcException &e) {
        ROS_ERROR_STREAM("error parsing: " << key);
        //xml.write(std::cerr); std::cerr << std::endl;
        throw (e);
      }
    }
 
  } // namespace
}  // namespace
