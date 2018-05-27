/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/simple_body.h"
#include "tagslam/yaml_utils.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>

namespace tagslam {
  using boost::irange;

  bool SimpleBody::parse(XmlRpc::XmlRpcValue body_defaults,
                         XmlRpc::XmlRpcValue body) {
    if (!RigidBody::parseCommon(body_defaults, body)) {
      return (false);
    }
    if (body.hasMember("tags")) {
      TagVec tv = Tag::parseTags(body["tags"], defaultTagSize);
      addTags(tv);
    }
    return (true);
  }

  bool SimpleBody::write(std::ostream &os, const std::string &prefix) const {
    // write common section
    if (!RigidBody::writeCommon(os, prefix)) {
      return (false);
    }
    const std::string ind = prefix + "    "; // indent
    os << ind << "tags: " << std::endl;
    PoseNoise smallNoise = makePoseNoise(0.001, 0.001);
    for (const auto &tm: tags) {
      const auto &tag = tm.second;
      os << ind << "- id: "   << tag->id << std::endl;
      os << ind << "  size: " << tag->size << std::endl;
      if (tag->poseEstimate.isValid()) {
        yaml_utils::write_pose(os, ind + "  ", tag->poseEstimate, smallNoise, true);
        //TODO: tag->poseEstimate.getNoise());
      }
    }
    return (true);
  }

}  // namespace
