/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag2.h"
#include "tagslam/yaml_utils.h"
#include <ros/ros.h>
#include <map>

using std::cout;
using std::endl;

namespace tagslam {
  static std::vector<Point3d> make_object_corners(double s) {
    const std::vector<Point3d> c =
      { Point3d(-s/2,-s/2, 0), Point3d( s/2,-s/2, 0),
        Point3d( s/2, s/2, 0), Point3d(-s/2, s/2, 0) };
    return (c);
  }

  Tag2::Tag2(int ida, int bts, double s, const PoseWithNoise &pn) :
    id(ida), bits(bts), size(s), poseWithNoise(pn)  {
    objectCorners = make_object_corners(size);
  }

  Point3d Tag2::getObjectCorner(int i) const {
    return objectCorners[i];
  }

  Tag2Vec Tag2::parseTags(XmlRpc::XmlRpcValue xmltags, double size) {
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
        tags.push_back(make(id, bits, sz, pn));
      } else {
        tags.push_back(make(id, bits, sz, PoseWithNoise()));
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

  Tag2Ptr Tag2::make(int tagId, int bits, double size, const PoseWithNoise &pn) {
    Tag2Ptr tagPtr(new Tag2(tagId, bits, size, pn));
    return (tagPtr);
  }
  
  std::ostream &operator<<(std::ostream &os, const Tag2 &tag) {
    os << tag.id << " sz: " << tag.size << " " << tag.poseWithNoise;
    return (os);
  }
}  // namespace
