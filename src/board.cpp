/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/board.h"
#include "tagslam/yaml_utils.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>

namespace tagslam {
  using boost::irange;
  using yaml_utils::parse;
  bool Board::parse(XmlRpc::XmlRpcValue body_defaults,
                    XmlRpc::XmlRpcValue body) {
    if (!RigidBody::parseCommon(body_defaults, body)) {
      return (false);
    }
    try {
      XmlRpc::XmlRpcValue board = body[type];
      tagStartId = yaml_utils::parse<int>(board, "tag_start_id", -1);
      tagSize = yaml_utils::parse<double>(board, "tag_size", -1.0);
      tagSpacing = yaml_utils::parse<double>(board, "tag_spacing", 0.25);
      tagRows    = yaml_utils::parse<int>(board, "tag_rows", -1);
      tagColumns = yaml_utils::parse<int>(board, "tag_columns", -1);
      tagRotationNoise = yaml_utils::parse<double>(board, "tag_rotation_noise", 0.0);
      tagPositionNoise = yaml_utils::parse<double>(board, "tag_position_noise", 0.0);
    } catch (const XmlRpc::XmlRpcException &e) {
      throw std::runtime_error("error parsing board of body: " + name);
    }
    if (tagRows < 0 || tagColumns < 0) {
      throw std::runtime_error("must specify tag rows and cols for board body: " + name);
    }
    if (tagStartId < 0 || tagSize < 0) {
      throw std::runtime_error("must specify tag start id and size for board body: " + name);
    }
    int tagid = tagStartId;
    for (int row = 0; row < tagRows; row++) {
      for (int col = 0; col < tagColumns; col++) {
        gtsam::Pose3 pose(gtsam::Rot3(),
                          gtsam::Point3(col * tagSize * (1.0 + tagSpacing),
                                        row * tagSize * (1.0 + tagSpacing), 0.0));
        PoseNoise noise = makePoseNoise(tagRotationNoise, tagPositionNoise);
        PoseEstimate pe(pose, 0.0, 0, noise);
        addTag(Tag::makeTag(tagid++, tagBits, tagSize, pe, true));
      }
    }
    return (true);
  }

  bool Board::write(std::ostream &os, const std::string &prefix) const {
    // write common section
    if (!RigidBody::writeCommon(os, prefix)) {
      return (false);
    }
    os << prefix + "    " << type << ":" << std::endl;
    const std::string ind = prefix + "      "; // indent
    os << ind << "tag_start_id: "  << tagStartId << std::endl;
    os << ind << "tag_size: " << tagSize << std::endl;
    os << ind << "tag_bits: " << tagBits << std::endl;
    os << ind << "tag_spacing: " << tagSpacing << std::endl;
    os << ind << "tag_rows: " << tagRows << std::endl;
    os << ind << "tag_columns: " << tagColumns << std::endl;
    os << ind << "tag_rotation_noise: " << tagRotationNoise << std::endl;
    os << ind << "tag_position_noise: " << tagPositionNoise << std::endl;
    return (true);
  }


}  // namespace
