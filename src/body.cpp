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

#include <yaml-cpp/yaml.h>

#include <tagslam/board.hpp>
#include <tagslam/body.hpp>
#include <tagslam/body_defaults.hpp>
#include <tagslam/logging.hpp>
#include <tagslam/simple_body.hpp>
#include <tagslam/staggered_board.hpp>
#include <tagslam/yaml.hpp>
#include <tagslam/yaml_utils.hpp>

namespace tagslam
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("body")); }

static int body_id = 0;

static BodyPtr make_type(const std::string & name, const std::string & type)
{
  BodyPtr p;
  if (type == "board") {
    BoardPtr board(new Board(name));
    p = board;
  } else if (type == "staggered_board") {
    p.reset(new StaggeredBoard(name));
  } else if (type == "simple") {
    SimpleBodyPtr sb(new SimpleBody(name));
    p = sb;
  } else {
    if (type == "camera_rig") {
      LOG_ERROR("camera_rig is deprecated, change to simple!");
    }
    BOMB_OUT("invalid rigid body type: " + type);
  }
  p->setType(type);
  p->setId(body_id++);
  return (p);
}

bool Body::parseCommon(const YAML::Node & body)
{
  try {
    isStatic_ = yaml::parse<bool>(body, "is_static");
    defaultTagSize_ = yaml::parse<double>(body, "default_tag_size", 0.0);
    maxHammingDistance_ = yaml::parse<int>(body, "max_hamming_distance", 2);
    overrideTagPositionNoise_ =
      yaml::parse<double>(body, "override_tag_position_noise", -1.0);
    overrideTagRotationNoise_ =
      yaml::parse<double>(body, "override_tag_rotation_noise", -1.0);
    T_body_odom_ =
      yaml::parse<Transform>(body, "T_body_odom", Transform::Identity());
    odomFrameId_ = yaml::parse<string>(body, "odom_frame_id", "");
    odomTopic_ = yaml::parse<string>(body, "odom_topic", "");
    odomTranslationNoise_ =
      yaml::parse<double>(body, "odom_translation_noise", -1.0);
    odomRotationNoise_ = yaml::parse<double>(body, "odom_rotation_noise", -1.0);
    publishCovariance_ = yaml::parse<bool>(body, "publish_covariance", true);
    // first read old tag, then new one if provided
    const double oanm = yaml::parse<double>(body, "odom_acceleration", 5.0);
    odomAccelerationNoiseMin_ =
      yaml::parse<double>(body, "odom_acceleration_noise_min", oanm);

    const double oaanm =
      yaml::parse<double>(body, "odom_angular_acceleration", 5.0);
    odomAngularAccelerationNoiseMin_ =
      yaml::parse<double>(body, "odom_angular_acceleration_noise_min", oaanm);

    odomAccelerationNoiseMax_ = yaml::parse<double>(
      body, "odom_acceleration_noise_max", 10 * odomAccelerationNoiseMin_);
    odomAngularAccelerationNoiseMax_ = yaml::parse<double>(
      body, "odom_angular_acceleration_noise_max",
      10 * odomAngularAccelerationNoiseMin_);

    ignoreTags_ = yaml::parse_container<std::set<int>>(
      body, "ignore_tags", std::set<int>());
    poseWithNoise_ = yaml::parse<PoseWithNoise>(body, "pose", PoseWithNoise());
    if (poseWithNoise_.isValid() && !isStatic_) {
      BOMB_OUT("body " << getName() << " is dynamic but has pose!");
    }
    fakeOdomTranslationNoise_ =
      yaml::parse<double>(body, "fake_odom_translation_noise", -1.0);
    fakeOdomRotationNoise_ =
      yaml::parse<double>(body, "fake_odom_rotation_noise", -1.0);
  } catch (const YAML::RepresentationException & e) {
    BOMB_OUT("error parsing body: " << name_);
  }
  if (!isStatic_ && poseWithNoise_.isValid()) {
    BOMB_OUT("dynamic body has prior pose: " + name_);
  }
  return (true);
}

BodyPtr Body::parse_body(const string & name, const YAML::Node & body)
{
  const string type = yaml::parse<string>(body, "type");
  const BodyPtr rb = make_type(name, type);
  rb->parseCommon(body);
  rb->parse(body, rb);
  return (rb);
}

TagPtr Body::findTag(int tagId, int bits) const
{
  const auto it = tags_.find(tagId);
  return (
    (it == tags_.end() || it->second->getBits() != bits) ? NULL : it->second);
}

void Body::addTag(const TagPtr & tag)
{
  tags_.insert(TagMap::value_type(tag->getId(), tag));
  tagList_.push_back(tag);
}

void Body::addTags(const TagVec & tags)
{
  for (const auto & tag : tags) {
    addTag(tag);
  }
}

BodyVec Body::parse_bodies(const YAML::Node & config)
{
  BodyVec bv;
  if (!config["bodies"]) {
    BOMB_OUT("no bodies found!");
  }
  const auto bodies = config["bodies"];
  for (const auto & body : bodies) {
    if (!body.IsMap()) continue;
    std::cout << "body: " << body << std::endl;
    /*
XXX fix me! 
    for (const auto bi : body) {
      if (bi.IsScalar()) {
        BodyPtr rb = parse_body(bi, it->second);
        bv.push_back(rb);
      }
    }
*/
  }
  return (bv);
}

bool Body::writeCommon(std::ostream & os, const string & prefix) const
{
  os << prefix << "- " << name_ << ":" << std::endl;
  string pfix = prefix + "    ";
  os << pfix << "type: " << type_ << std::endl;
  os << pfix << "is_static: " << (isStatic_ ? "true" : "false") << std::endl;
  os << pfix << "default_tag_size: " << defaultTagSize_ << std::endl;
  return (true);
}

}  // namespace tagslam
