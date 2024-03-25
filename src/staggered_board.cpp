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

#include <tagslam/logging.hpp>
#include <tagslam/staggered_board.hpp>
#include <tagslam/yaml.hpp>

namespace tagslam
{
static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("staggered_board"));
}

bool StaggeredBoard::parse(const YAML::Node & body, const BodyPtr & bp)
{
  Transform T_board_body = Transform::Identity();  // defaults to identity
  try {
    const auto board = body[type_];
    tagStartId_ = yaml::parse<int>(board, "tag_start_id");
    tagSize_ = yaml::parse<double>(board, "tag_size");
    tagSpacing_ = yaml::parse<double>(board, "tag_spacing");
    tagRows_ = yaml::parse<int>(board, "tag_rows");
    tagColumns_ = yaml::parse<int>(board, "tag_columns");
    tagBits_ = yaml::parse<int>(board, "tag_bits", 6);
    tagRotationNoise_ = yaml::parse<double>(board, "tag_rotation_noise", 0.0);
    tagPositionNoise_ = yaml::parse<double>(board, "tag_position_noise", 0.0);
    if (board["pose"]) {
      T_board_body = yaml::parse<Transform>(board, "pose");
    }
  } catch (const std::runtime_error & e) {
    BOMB_OUT("error parsing board of body: " + name_);
  }
  int tagid = tagStartId_;
  for (int row = 0; row < tagRows_; row++) {
    for (int col = row % 2; col < tagColumns_; col += 2) {
      const Point3d center(
        col * tagSize_ * (1.0 + tagSpacing_),
        -row * tagSize_ * (1.0 + tagSpacing_), 0.0);
      const Transform pose =
        T_board_body.inverse() * make_transform(Point3d(0, 0, 0), center);
      const PoseNoise noise =
        PoseNoise::make(tagRotationNoise_, tagPositionNoise_);
      const PoseWithNoise pn(pose, noise, true);
      addTag(Tag::make(tagid++, tagBits_, tagSize_, pn, bp));
    }
  }
  return (true);
}

bool StaggeredBoard::write(std::ostream & os, const string & prefix) const
{
  // write common section
  if (!Body::writeCommon(os, prefix)) {
    return (false);
  }
  os << prefix + "    " << type_ << ":" << std::endl;
  const string ind = prefix + "      ";  // indent
  os << ind << "tag_start_id: " << tagStartId_ << std::endl;
  os << ind << "tag_size: " << tagSize_ << std::endl;
  os << ind << "tag_bits: " << tagBits_ << std::endl;
  os << ind << "tag_spacing: " << tagSpacing_ << std::endl;
  os << ind << "tag_rows: " << tagRows_ << std::endl;
  os << ind << "tag_columns: " << tagColumns_ << std::endl;
  os << ind << "tag_rotation_noise: " << tagRotationNoise_ << std::endl;
  os << ind << "tag_position_noise: " << tagPositionNoise_ << std::endl;
  return (true);
}

}  // namespace tagslam
