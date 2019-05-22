/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include "tagslam/geometry.h"
#include "tagslam/pose_with_noise.h"

#include <ros/ros.h>
#include <XmlRpcException.h>

#include <boost/range/irange.hpp>

#include <vector>
#include <string>

namespace tagslam {
  namespace xml {
    //
    // must template the cast to work around default cast bombing out
    // for doubles when getting "0" vs "0.0"
    //
    template <typename T>
    T cast(XmlRpc::XmlRpcValue xml) {
      return (static_cast<T>(xml));
    }
    //
    // specialize cast for double to work around sillyness
    //
    template <>
    inline double cast<double>(XmlRpc::XmlRpcValue xml) {
      try {
        // first try casting to double
        return (static_cast<double>(xml));
      } catch (const XmlRpc::XmlRpcException &e) {
        // if fails, try casting to int
        return ((double)static_cast<int>(xml));
      }
    }
    
    //
    // parse without default (throws exception if not found!), e.g:
    // int x = parse<int>(xmlrpcvalue, "foo");
    // 
    template <typename T>
    T parse(XmlRpc::XmlRpcValue xml, const std::string &key) {
      if (!xml.hasMember(key)) {
        ROS_ERROR_STREAM("key not found: " << key);
        throw XmlRpc::XmlRpcException("key not found: " + key);
      }
      try {
        return (cast<T>(xml[key]));
      } catch (const XmlRpc::XmlRpcException &e) {
        ROS_ERROR_STREAM("error parsing: " << key);
        throw (e);
      } 
    }

    // specialization for reading 3d points

    template <>
    Point3d parse(XmlRpc::XmlRpcValue xml, const std::string &key);


    // specialization for reading transforms

    template <>
    Transform parse(XmlRpc::XmlRpcValue xml, const std::string &key);
 
    // specialization for reading noise
 
    template <>
    PoseNoise parse(XmlRpc::XmlRpcValue xml, const std::string &key);

    // specialization for reading pose with noise

    template <>
    PoseWithNoise parse(XmlRpc::XmlRpcValue xml, const std::string &key);

    // specialization for reading ros::Time (must be enclosed in ""!)

    template<>
    ros::Time parse(XmlRpc::XmlRpcValue xml, const std::string &key);

    //
    // parse with default, for example
    // int x = parse<int>(xmlrpcvalue, "foo", 0.0);
    // 
    template <typename T>
    T parse(XmlRpc::XmlRpcValue xml, const std::string &key, const T &def) {
      if (xml.hasMember(key)) {
        return (parse<T>(xml, key));
      }
      return (def);
    }

    //
    // a version for parsing containers
    //
    template<typename C>
    C parse_container(XmlRpc::XmlRpcValue xml, const std::string &key) {
      if (!xml.hasMember(key)) {
        ROS_ERROR_STREAM("key not found: " << key);
        xml.write(std::cerr); std::cerr << std::endl;
        throw XmlRpc::XmlRpcException("key not found: " + key);
      }

      C v;
      try {
        if (xml[key].getType() != XmlRpc::XmlRpcValue::TypeArray) {
          ROS_ERROR_STREAM("must be array: " << key);
          xml[key].write(std::cerr); std::cerr << std::endl;
          throw XmlRpc::XmlRpcException("must be array: " + key);
        }
        for (const auto i: boost::irange(0, xml[key].size())) {
          v.insert(v.end(), cast<typename C::value_type>(xml[key][i]));
        }
      } catch (const XmlRpc::XmlRpcException &e) {
        ROS_ERROR_STREAM("error parsing container: " << key);
        xml[key].write(std::cerr); std::cerr << std::endl;
        throw (e);
      } 
      return (v);
    }

    // parsing containers with default
    
    template<typename C>
    C parse_container(XmlRpc::XmlRpcValue xml, const std::string &key,
                 const C &def) {
      if (!xml.hasMember(key)) {
        return (def);
      }
      return (parse_container<C>(xml, key));
    }

  }
}
