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

#include <tagslam/logging.hpp>
#include <tagslam/yaml.hpp>

namespace tagslam
{
namespace yaml
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("yaml")); }

template <>
Point3d parse(const YAML::Node & conf, const std::string & key)
{
  if (!conf[key]) {
    LOG_ERROR("key not found: " << key);
    throw(std::runtime_error("key not found: " + key));
  }
  try {
    const Point3d v(
      parse<double>(conf[key], "x"), parse<double>(conf[key], "y"),
      parse<double>(conf[key], "z"));
    return (v);
  } catch (const std::runtime_error & e) {
    LOG_ERROR("error parsing: " << key);
    throw(e);
  }
}

template <>
Transform parse(const YAML::Node & conf, const std::string & key)
{
  if (!conf[key]) {
    LOG_ERROR("key not found: " << key);
    throw(std::runtime_error("key not found: " + key));
  }
  try {
    const Point3d p = parse<Point3d>(conf[key], "position");
    const Point3d r = parse<Point3d>(conf[key], "rotation");
    return (make_transform(r, p));
  } catch (const std::runtime_error & e) {
    LOG_ERROR("error parsing: " << key);
    throw(e);
  }
}

template <>
PoseNoise parse(const YAML::Node & conf, const std::string & key)
{
  if (!conf[key]) {
    LOG_ERROR("key not found: " << key);
    throw std::runtime_error("key not found: " + key);
  }
  try {
    if (conf[key]["position_noise"] && conf[key]["rotation_noise"]) {
      const Point3d p = parse<Point3d>(conf[key], "position_noise");
      const Point3d r = parse<Point3d>(conf[key], "rotation_noise");
      return (PoseNoise::make(r, p));
    } else if (conf[key]["R"]) {
      auto Rd = parse_container<std::vector<double>>(conf[key], "R");
      const auto R = Eigen::Map<Eigen::Matrix<double, 6, 6>>(&Rd[0]);
      return (PoseNoise::makeFromR(R));
    } else {
      throw std::runtime_error("no valid noise for: " + key);
    }
  } catch (const std::runtime_error & e) {
    LOG_ERROR("error parsing: " << key);
    throw(e);
  }
}

template <>
PoseWithNoise parse(const YAML::Node & conf, const std::string & key)
{
  if (!conf[key]) {
    LOG_ERROR("key not found: " << key);
    throw std::runtime_error("key not found: " + key);
  }
  try {
    const Transform pose = parse<Transform>(conf, key);
    const PoseNoise noise = parse<PoseNoise>(conf, key);
    return (PoseWithNoise(pose, noise, true));
  } catch (const std::runtime_error & e) {
    LOG_ERROR("error parsing: " << key);
    //xml.write(std::cerr); std::cerr << std::endl;
    throw(e);
  }
}
template <>
uint64_t parse(const YAML::Node & conf, const std::string & key)
{
  if (!conf[key]) {
    LOG_ERROR("key not found: " << key);
    throw std::runtime_error("key not found: " + key);
  }
  try {
    const std::string s = conf[key].as<std::string>();
    size_t pos = s.find(".", 0);
    if (pos == std::string::npos) {
      BOMB_OUT("bad ros time value: " << s);
    }
    const std::string nsec = s.substr(pos + 1, std::string::npos);
    const std::string sec = s.substr(0, pos);
    if (nsec.size() != 9) {
      BOMB_OUT("ros nsec length is not 9 but: " << nsec.size());
    }
    const uint64_t t = std::stoi(sec) * 1000000000ULL + std::stoi(nsec);
    return (t);
  } catch (const std::runtime_error & e) {
    LOG_ERROR("error parsing: " << key);
    //conf.write(std::cerr); std::cerr << std::endl;
    throw(e);
  }
}
}  // namespace yaml
}  // namespace tagslam
