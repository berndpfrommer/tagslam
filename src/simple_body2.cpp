/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/simple_body2.h"
#include "tagslam/yaml_utils.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>

namespace tagslam {
  using boost::irange;

  bool SimpleBody2::parse(XmlRpc::XmlRpcValue body) {
    if (body.hasMember("tags")) {
      Tag2Vec tv = Tag2::parseTags(body["tags"], defaultTagSize);
      addTags(tv);
    }
    return (true);
  }

  bool SimpleBody2::write(std::ostream &os, const std::string &prefix) const {
    // write common section
    if (!Body::writeCommon(os, prefix)) {
      return (false);
    }
    const std::string ind = prefix + "    "; // indent
    os << ind << "tags: " << std::endl;
    PoseNoise2 smallNoise = PoseNoise2::make(0.001, 0.001);
    for (const auto &tm: tags) {
      const auto &tag = tm.second;
      os << ind << "- id: "   << tag->getId() << std::endl;
      os << ind << "  size: " << tag->getSize() << std::endl;
      if (tag->getPoseWithNoise().isValid()) {
        yaml_utils::write_pose(os, ind + "  ", tag->getPoseWithNoise().getPose(),
                               smallNoise, true);
        //TODO: tag->poseEstimate.getNoise());
      }
    }
    return (true);
  }

}  // namespace
