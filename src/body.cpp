/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/body.h"
#include "tagslam/logging.h"
#include "tagslam/simple_body2.h"
#include "tagslam/board2.h"
#include "tagslam/body_defaults.h"
#include "tagslam/yaml_utils.h"
#include "tagslam/xml.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>


namespace tagslam {
  using boost::irange;

  static int body_id = 0;

  static BodyPtr make_type(const std::string &name,
                                const std::string &type) {
    BodyPtr p;
    if (type == "board") {
      Board2Ptr board(new Board2(name));
      p = board;
    } else  if (type == "simple" || type == "camera_rig") {
      SimpleBody2Ptr sb(new SimpleBody2(name));
      p = sb;
    } else {
      BOMB_OUT("invalid rigid body type: " + type);
    }
    p->setType(type);
    p->setId(body_id++);
    return (p);
  }

  bool Body::parseCommon(XmlRpc::XmlRpcValue body) {
    try {
      //const double def_pos_noise = BodyDefaults::instance()->positionNoise;
      //const double def_rot_noise = BodyDefaults::instance()->rotationNoise;
      isStatic_       = xml::parse<bool>(body,  "is_static");
      defaultTagSize_ = xml::parse<double>(body, "default_tag_size", 0.0);
      maxHammingDistance_ = xml::parse<int>(body, "max_hamming_distance", 2);
      overrideTagPositionNoise_ =
        xml::parse<double>(body, "override_tag_position_noise", -1.0);
      overrideTagRotationNoise_ =
        xml::parse<double>(body, "override_tag_rotation_noise", -1.0);
      T_body_odom_      = xml::parse<Transform>(body,  "T_body_odom",
                                                Transform::Identity());
      odomFrameId_      = xml::parse<std::string>(body, "odom_frame_id", "");
      odomTopic_        = xml::parse<std::string>(body, "odom_topic", "");
      odomAcceleration_ = xml::parse<double>(body, "odom_acceleration", 5.0);
      odomAngularAcceleration_=
        xml::parse<double>(body, "odom_angular_acceleration", 5.0);
      ignoreTags_ = xml::parse_container<std::set<int>>(body, "ignore_tags",
                                                        std::set<int>());
      poseWithNoise_ = xml::parse<PoseWithNoise>(body, "pose",
                                                 PoseWithNoise());
      if (poseWithNoise_.isValid() && !isStatic_) {
        BOMB_OUT("body " << getName() << " is dynamic but has pose!");
      }
    } catch (const XmlRpc::XmlRpcException &e) {
      BOMB_OUT("error parsing body: " << name);
    }
    if (!isStatic_ && poseWithNoise_.isValid()) {
      BOMB_OUT("dynamic body has prior pose: " + name);
    }
    return (true);
  }

  BodyPtr
  Body::parse_body(const std::string &name,  XmlRpc::XmlRpcValue body) {
    const std::string type = xml::parse<std::string>(body, "type");
    const BodyPtr rb = make_type(name, type);
    rb->parseCommon(body);
    rb->parse(body, rb);
    return (rb);
  }

  Tag2Ptr Body::findTag(int tagId, int bits) const {
    const auto it = tags.find(tagId);
    return ((it == tags.end() || it->second->getBits() != bits)?
            NULL: it->second);
  }

  void Body::addTag(const Tag2Ptr &tag) {
    tags.insert(Tag2Map::value_type(tag->getId(), tag));
  }

  void Body::addTags(const Tag2Vec &tags) {
    for (const auto &tag:tags) {
      addTag(tag);
    }
  }

  std::list<Tag2ConstPtr> Body::getTags() const {
    std::list<Tag2ConstPtr> taglist;
    for (const auto &tag_it: tags) {
      taglist.push_back(tag_it.second);
    }
    return (taglist);
  }

  BodyVec
  Body::parse_bodies(XmlRpc::XmlRpcValue config) {
    BodyVec bv;
    if (!config.hasMember("bodies")) {
      BOMB_OUT("no bodies found!");
    }
    XmlRpc::XmlRpcValue bodies = config["bodies"];
    for (const auto i: irange(0, bodies.size())) {
      if (bodies[i].getType() !=
          XmlRpc::XmlRpcValue::TypeStruct) continue;
      for (XmlRpc::XmlRpcValue::iterator it = bodies[i].begin();
           it != bodies[i].end(); ++it) {
        if (it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
          continue;
        }
        BodyPtr rb = parse_body(it->first, it->second);
        bv.push_back(rb);
      }
    }
    return (bv);
  }

  bool Body::writeCommon(std::ostream &os, const std::string &prefix) const {
    os << prefix << "- " << name << ":" << std::endl;
    std::string pfix = prefix + "    ";
    os << pfix << "type: " << type << std::endl;
    os << pfix << "is_static: " <<
      (isStatic_ ? "true" : "false") << std::endl;
    os << pfix << "default_tag_size: " << defaultTagSize_ << std::endl;
    if (isStatic_) {
      os << pfix << "pose:" << std::endl;;
      PoseNoise2 smallNoise = PoseNoise2::make(0.001, 0.001);
      yaml_utils::write_pose(os, pfix + "  ", poseWithNoise_.getPose(),
                             smallNoise, poseWithNoise_.isValid());
      // TODO: poseEstimate.getNoise());
    }
    return (true);
  }

}  // namespace
