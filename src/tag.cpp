/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag.h"
#include "tagslam/body.h"
#include "tagslam/xml.h"
#include "tagslam/logging.h"
#include <ros/ros.h>
#include <map>

using std::cout;
using std::endl;

namespace tagslam {
  Tag::Tag(int ida, int bts, double s, const PoseWithNoise &pn,
             const std::shared_ptr<Body> &body) :
    id_(ida), bits_(bts), size_(s), poseWithNoise_(pn), body_(body)  {
    objectCorners_ <<
      -s/2, -s/2, 0,
       s/2, -s/2, 0,
       s/2,  s/2, 0,
      -s/2,  s/2, 0;
    if (poseWithNoise_.isValid() && body->overrides()) {
      poseWithNoise_.setNoise(
        PoseNoise::make(body->getOverrideTagRotationNoise(),
                         body->getOverrideTagPositionNoise()));
    }
  }

  TagVec Tag::parseTags(XmlRpc::XmlRpcValue xmltags, double size,
                          const std::shared_ptr<Body> &body) {
    std::vector<TagPtr> tags;
    for (uint32_t i = 0; i < (unsigned int) xmltags.size(); i++) {
      if (xmltags[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;
      const int id    = xml::parse<int>(xmltags[i],    "id");
      const int bits  = xml::parse<int>(xmltags[i],    "bits", 6);
      const double sz = xml::parse<double>(xmltags[i], "size", size);
      const PoseWithNoise pwn =
        xml::parse<PoseWithNoise>(xmltags[i], "pose", PoseWithNoise());
      tags.push_back(make(id, bits, sz, pwn, body));
    }
    //std::cout << "-------tags: " << std::endl;
    //for (const auto &tag:tags) {
    //std::cout << *tag << std::endl;
    //}
    return (tags);
  }

  TagPtr Tag::make(int tagId, int bits, double size, const PoseWithNoise &pn,
                     const std::shared_ptr<Body> &body) {
    if (size < 1e-6) {
      BOMB_OUT("making tag " << tagId << " with zero size!");
    }
    TagPtr tagPtr(new Tag(tagId, bits, size, pn, body));
    return (tagPtr);
  }
  
  std::ostream &operator<<(std::ostream &os, const Tag &tag) {
    os << tag.id_ << " sz: " << tag.size_ << " " << tag.body_->getName()
       << " " << tag.poseWithNoise_;
    return (os);
  }
}  // namespace
