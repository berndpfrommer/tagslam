/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/board2.h"
#include "tagslam/yaml_utils.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>

namespace tagslam {
  using boost::irange;
  using yaml_utils::parse;
  bool Board2::parse(XmlRpc::XmlRpcValue body) {
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
        Point3d center(col * tagSize * (1.0 + tagSpacing),
                       row * tagSize * (1.0 + tagSpacing), 0.0);
        Transform pose = make_transform(Point3d(0,0,0), center);
        PoseNoise2 noise = PoseNoise2::make(tagRotationNoise, tagPositionNoise);
        PoseWithNoise pn(pose, noise, true);
        addTag(Tag2::make(tagid++, tagBits, tagSize, pn));
      }
    }
    return (true);
  }

  bool Board2::write(std::ostream &os, const std::string &prefix) const {
    // write common section
    if (!Body::writeCommon(os, prefix)) {
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
