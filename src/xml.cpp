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
        //xml.write(std::cerr); std::cerr << std::endl;
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
        //xml.write(std::cerr); std::cerr << std::endl;
        throw (e);
      }
    }

    template <>
    Transform parse(XmlRpc::XmlRpcValue xml) {
      try {
        const Point3d p = parse<Point3d>(xml, "position");
        const Point3d r = parse<Point3d>(xml, "rotation");
        return (make_transform(r, p));
      } catch (const XmlRpc::XmlRpcException &e) {
        ROS_ERROR_STREAM("error parsing transform");
        //xml.write(std::cerr); std::cerr << std::endl;
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
        const Point3d p = parse<Point3d>(xml[key], "position_noise");
        const Point3d r = parse<Point3d>(xml[key], "rotation_noise");
        return (PoseNoise2::make(r, p));
      } catch (const XmlRpc::XmlRpcException &e) {
        ROS_ERROR_STREAM("error parsing: " << key);
        //xml.write(std::cerr); std::cerr << std::endl;
        throw (e);
      }
    }
  
    template <>
    PoseNoise2 parse(XmlRpc::XmlRpcValue xml) {
      try {
        const Point3d p = parse<Point3d>(xml, "position_noise");
        const Point3d r = parse<Point3d>(xml, "rotation_noise");
        return (PoseNoise2::make(r, p));
      } catch (const XmlRpc::XmlRpcException &e) {
        ROS_ERROR_STREAM("error parsing pose noise ");
        //xml.write(std::cerr); std::cerr << std::endl;
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

    // specialization for reading pose with noise without key

    template<>
    PoseWithNoise parse(XmlRpc::XmlRpcValue xml) {
      try {
        const Transform  pose  = parse<Transform>(xml);
        const PoseNoise2 noise = parse<PoseNoise2>(xml);
        return (PoseWithNoise(pose, noise, true));
      } catch (const XmlRpc::XmlRpcException &e) {
        ROS_ERROR_STREAM("error parsing pose with noise!");
        //xml.write(std::cerr); std::cerr << std::endl;
        throw (e);
      }
    }
 
    // read pose with noise with key
    template<>
    PoseWithNoise parse(XmlRpc::XmlRpcValue xml, const PoseWithNoise &def) {
      try {
        return (parse<PoseWithNoise>(xml));
      } catch (const XmlRpc::XmlRpcException &e) {
        return (def);
      } 
    }


  } // namespace
}  // namespace
