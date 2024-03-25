// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TAGSLAM__YAML_HPP_
#define TAGSLAM__YAML_HPP_

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tagslam/geometry.hpp>
#include <tagslam/logging.hpp>
#include <tagslam/pose_with_noise.hpp>
#include <vector>

namespace tagslam
{
namespace yaml
{
//
// must template the cast to work around default cast bombing out
// for doubles when getting "0" vs "0.0"
//
template <typename T>
T cast(const YAML::Node & node)
{
  return (node.as<T>());
}
//
// specialize cast in case we need to work around
// exceptions when getting 0 instead of 0.0

template <>
inline double cast<double>(const YAML::Node & node)
{
  return (node.as<double>());
}

//
// parse without default (throws exception if not found!), e.g:
// int x = parse<int>(node, "foo");
//
template <typename T>
T parse(const YAML::Node & node, const std::string & key)
{
  if (!node[key]) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("yaml"), "key not found: " << key);
    throw(std::runtime_error("key not found: " + key));
  }
  try {
    return (cast<T>(node[key]));
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("yaml"), "error parsing: " << key);
    throw(e);
  }
}

// specialization for reading 3d points

template <>
Point3d parse(const YAML::Node & node, const std::string & key);

// specialization for reading transforms

template <>
Transform parse(const YAML::Node & node, const std::string & key);

// specialization for reading noise

template <>
PoseNoise parse(const YAML::Node & node, const std::string & key);

// specialization for reading pose with noise

template <>
PoseWithNoise parse(const YAML::Node & node, const std::string & key);

// specialization for reading ros::Time (must be enclosed in ""!)
// XXX(Bernd) is this still needed?

template <>
uint64_t parse(const YAML::Node & node, const std::string & key);

//
// parse with default, for example
// int x = parse<int>(noderpcvalue, "foo", 0.0);
//
template <typename T>
T parse(const YAML::Node & node, const std::string & key, const T & def)
{
  if (node[key]) {
    return (parse<T>(node, key));
  }
  return (def);
}

//
// a version for parsing containers
//
template <typename C>
C parse_container(const YAML::Node & node, const std::string & key)
{
  if (!node[key]) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("yaml"), "key not found: " << key);
    std::cerr << node << std::endl;
    throw(std::runtime_error(key));
  }

  C v;
  try {
    if (!node[key].IsSequence()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yaml"), "must be array: " << key);
      std::cerr << node << std::endl;
      throw(std::runtime_error("must be array: " + key));
    }
    for (const auto & n : node[key]) {
      v.insert(v.end(), cast<typename C::value_type>(n));
    }
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("yaml"), "error parsing container: " << key);
    std::cerr << node[key] << std::endl;
    throw(e);
  }
  return (v);
}

// parsing containers with default

template <typename C>
C parse_container(
  const YAML::Node & node, const std::string & key, const C & def)
{
  if (!node[key]) {
    return (def);
  }
  return (parse_container<C>(node, key));
}

}  // namespace yaml
}  // namespace tagslam

#endif