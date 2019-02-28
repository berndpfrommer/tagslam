/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag2.h"
#include "tagslam/yaml_utils.h"
#include "tagslam/body.h"
#include <ros/ros.h>
#include <map>

using std::cout;
using std::endl;

namespace tagslam {
  static std::vector<Point3d, Eigen::aligned_allocator<Point3d>>
    make_object_corners(double s) {
    const std::vector<Point3d, Eigen::aligned_allocator<Point3d>> c =
      { Point3d(-s/2,-s/2, 0), Point3d( s/2,-s/2, 0),
        Point3d( s/2, s/2, 0), Point3d(-s/2, s/2, 0) };
    return (c);
  }

  Tag2::Tag2(int ida, int bts, double s, const PoseWithNoise &pn,
             const std::shared_ptr<Body> &body) :
    id_(ida), bits_(bts), size_(s), poseWithNoise_(pn), body_(body)  {
    objectCorners_ = make_object_corners(size_);
  }

  Point3d Tag2::getObjectCorner(int i) const {
    return objectCorners_[i];
  }

  Tag2Vec Tag2::parseTags(XmlRpc::XmlRpcValue xmltags, double size,
                          const std::shared_ptr<Body> &body) {
    std::vector<Tag2Ptr> tags;
    for (uint32_t i = 0; i < (unsigned int) xmltags.size(); i++) {
      if (xmltags[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;
      int id(0), bits(6);
      double sz(size);
      for (XmlRpc::XmlRpcValue::iterator it = xmltags[i].begin();
           it != xmltags[i].end(); ++it) {
        std::string field = it->first;
        if (field == "id") {          id   = static_cast<int>(it->second);
        } else if (field == "bits") { bits = static_cast<int>(it->second);
        } else if (field == "size") { sz   = static_cast<double>(it->second);
        }
      }
      Transform pose;
      PoseNoise2 noise;
      if (yaml_utils::get_pose_and_noise(xmltags[i], &pose, &noise)) {
        PoseWithNoise pn(pose, noise, true);
        tags.push_back(make(id, bits, sz, pn, body));
      } else {
        tags.push_back(make(id, bits, sz, PoseWithNoise(), body));
      }
    }
    /*
      std::cout << "-------tags: " << std::endl;
      for (const auto &tag:tags) {
      std::cout << *tag << std::endl;
      }
    */
    return (tags);
  }

  Tag2Ptr Tag2::make(int tagId, int bits, double size, const PoseWithNoise &pn,
                     const std::shared_ptr<Body> &body) {
    Tag2Ptr tagPtr(new Tag2(tagId, bits, size, pn, body));
    return (tagPtr);
  }
  
  std::ostream &operator<<(std::ostream &os, const Tag2 &tag) {
    os << tag.id_ << " sz: " << tag.size_ << " " << tag.body_->getName()
       << " " << tag.poseWithNoise_;
    return (os);
  }
}  // namespace
