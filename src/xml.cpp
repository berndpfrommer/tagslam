/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/xml.h"
#include "tagslam/logging.h"

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
    PoseNoise parse(XmlRpc::XmlRpcValue xml, const std::string &key) {
      if (!xml.hasMember(key)) {
        ROS_ERROR_STREAM("key not found: " << key);
        throw XmlRpc::XmlRpcException("key not found: " + key);
      }
      try {
        if (xml[key].hasMember("position_noise") &&
            xml[key].hasMember("rotation_noise")) {
          const Point3d p = parse<Point3d>(xml[key], "position_noise");
          const Point3d r = parse<Point3d>(xml[key], "rotation_noise");
          return (PoseNoise::make(r, p));
        } else if (xml[key].hasMember("R")) {
          auto Rd = parse_container<std::vector<double>>(xml[key], "R");
          const auto R = Eigen::Map<Eigen::Matrix<double, 6, 6> >(&Rd[0]);
          return (PoseNoise::makeFromR(R));
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
        const PoseNoise noise = parse<PoseNoise>(xml, key);
        return (PoseWithNoise(pose, noise, true));
      } catch (const XmlRpc::XmlRpcException &e) {
        ROS_ERROR_STREAM("error parsing: " << key);
        //xml.write(std::cerr); std::cerr << std::endl;
        throw (e);
      }
    }
    template<>
    ros::Time parse(XmlRpc::XmlRpcValue xml, const std::string &key) {
      if (!xml.hasMember(key)) {
        ROS_ERROR_STREAM("key not found: " << key);
        throw XmlRpc::XmlRpcException("key not found: " + key);
      }
      try {
        const std::string s = static_cast<std::string>(xml[key]);
        size_t pos = s.find(".", 0);
        if (pos == std::string::npos) {
          BOMB_OUT("bad ros time value: " << s);
        }
        const std::string nsec = s.substr(pos + 1, std::string::npos);
        const std::string sec  = s.substr(0, pos);
        if (nsec.size() != 9) {
          BOMB_OUT("ros nsec length is not 9 but: " << nsec.size());
        }
        ros::Time t(std::stoi(sec), std::stoi(nsec));
        return (t);
      } catch (const XmlRpc::XmlRpcException &e) {
        ROS_ERROR_STREAM("error parsing: " << key);
        //xml.write(std::cerr); std::cerr << std::endl;
        throw (e);
      }
    }
  } // namespace
}  // namespace
