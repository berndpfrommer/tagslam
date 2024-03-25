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

#include <tagslam/body.hpp>
#include <tagslam/logging.hpp>
#include <tagslam/tag.hpp>
#include <tagslam/yaml.hpp>

using std::cout;
using std::endl;

namespace tagslam
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("tag")); }

Tag::Tag(
  int ida, int bts, double s, const PoseWithNoise & pn,
  const std::shared_ptr<Body> & body)
: id_(ida), bits_(bts), size_(s), poseWithNoise_(pn), body_(body)
{
  objectCorners_ << -s / 2, -s / 2, 0, s / 2, -s / 2, 0, s / 2, s / 2, 0,
    -s / 2, s / 2, 0;
  if (poseWithNoise_.isValid() && body->overrides()) {
    poseWithNoise_.setNoise(PoseNoise::make(
      body->getOverrideTagRotationNoise(),
      body->getOverrideTagPositionNoise()));
  }
}

TagVec Tag::parseTags(
  const YAML::Node & conf, double size, const std::shared_ptr<Body> & body)
{
  std::vector<TagPtr> tags;
  if (!conf.IsSequence()) {
    BOMB_OUT("expected list of tags!");
  }
  for (const auto & tag : conf) {
    const int id = tag["id"].as<int>();
    const int bits = yaml::parse<int>(tag, "bits", 6);
    const double sz = yaml::parse<double>(tag, "size", size);
    const PoseWithNoise pwn =
      yaml::parse<PoseWithNoise>(tag, "pose", PoseWithNoise());
    tags.push_back(make(id, bits, sz, pwn, body));
  }
  std::cout << "-------tags: " << std::endl;
  for (const auto & tag : tags) {
    std::cout << *tag << std::endl;
  }
  return (tags);
}

TagPtr Tag::make(
  int tagId, int bits, double size, const PoseWithNoise & pn,
  const std::shared_ptr<Body> & body)
{
  if (size < 1e-6) {
    BOMB_OUT("making tag " << tagId << " with zero size!");
  }
  TagPtr tagPtr(new Tag(tagId, bits, size, pn, body));
  return (tagPtr);
}

std::ostream & operator<<(std::ostream & os, const Tag & tag)
{
  os << tag.id_ << " sz: " << tag.size_ << " " << tag.body_->getName() << " "
     << tag.poseWithNoise_;
  return (os);
}
}  // namespace tagslam
