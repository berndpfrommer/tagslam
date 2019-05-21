/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/board.h"
#include "tagslam/xml.h"
#include "tagslam/logging.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>

namespace tagslam {
  using boost::irange;
  bool Board::parse(XmlRpc::XmlRpcValue body, const BodyPtr &bp) {
    try {
      const XmlRpc::XmlRpcValue board = body[type_];
      tagStartId_ = xml::parse<int>(board, "tag_start_id");
      tagSize_    = xml::parse<double>(board, "tag_size");
      tagSpacing_ = xml::parse<double>(board, "tag_spacing");
      tagRows_    = xml::parse<int>(board, "tag_rows");
      tagColumns_ = xml::parse<int>(board, "tag_columns");
      tagBits_    = xml::parse<int>(board, "tag_bits", 6);
      tagRotationNoise_ = xml::parse<double>(board, "tag_rotation_noise", 0.0);
      tagPositionNoise_ = xml::parse<double>(board, "tag_position_noise", 0.0);
    } catch (const XmlRpc::XmlRpcException &e) {
      BOMB_OUT("error parsing board of body: " + name_);
    }
    int tagid = tagStartId_;
    for (const int row: irange(0, tagRows_)) {
      for (const int col: irange(0, tagColumns_)) {
        const Point3d center(col * tagSize_ * (1.0 + tagSpacing_),
                             row * tagSize_ * (1.0 + tagSpacing_), 0.0);
        const Transform pose = make_transform(Point3d(0,0,0), center);
        const PoseNoise noise =
          PoseNoise::make(tagRotationNoise_, tagPositionNoise_);
        const PoseWithNoise pn(pose, noise, true);
        addTag(Tag::make(tagid++, tagBits_, tagSize_, pn, bp));
      }
    }
    return (true);
  }

  bool Board::write(std::ostream &os, const string &prefix) const {
    // write common section
    if (!Body::writeCommon(os, prefix)) {
      return (false);
    }
    os << prefix + "    " << type_ << ":" << std::endl;
    const string ind = prefix + "      "; // indent
    os << ind << "tag_start_id: "  << tagStartId_ << std::endl;
    os << ind << "tag_size: " << tagSize_ << std::endl;
    os << ind << "tag_bits: " << tagBits_ << std::endl;
    os << ind << "tag_spacing: " << tagSpacing_ << std::endl;
    os << ind << "tag_rows: " << tagRows_ << std::endl;
    os << ind << "tag_columns: " << tagColumns_ << std::endl;
    os << ind << "tag_rotation_noise: " << tagRotationNoise_ << std::endl;
    os << ind << "tag_position_noise: " << tagPositionNoise_ << std::endl;
    return (true);
  }


}  // namespace
