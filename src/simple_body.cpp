/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/simple_body.h"
#include "tagslam/yaml_utils.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>

namespace tagslam {
  using boost::irange;

  bool SimpleBody::parse(XmlRpc::XmlRpcValue body, const BodyPtr &bp) {
    if (body.hasMember("tags")) {
      TagVec tv = Tag::parseTags(body["tags"], defaultTagSize_, bp);
      addTags(tv);
    }
    return (true);
  }

  bool SimpleBody::write(std::ostream &os, const std::string &prefix) const {
    // write common section
    if (!Body::writeCommon(os, prefix)) {
      return (false);
    }
/*
    // Don't write tag poses here anymore
    const std::string ind = prefix + "    "; // indent
    os << ind << "tags: " << std::endl;
    PoseNoise smallNoise = PoseNoise::make(0.001, 0.001);
    for (const auto &tm: tags_) {
      const auto &tag = tm.second;
      os << ind << "- id: "   << tag->getId() << std::endl;
      os << ind << "  size: " << tag->getSize() << std::endl;
      if (tag->getPoseWithNoise().isValid()) {
        yaml_utils::write_pose(os, ind + "  ",
                               tag->getPoseWithNoise().getPose(),
                               smallNoise, true);
      }
    }
*/
    return (true);
  }

}  // namespace
