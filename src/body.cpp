/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/body.h"
#include "tagslam/simple_body2.h"
#include "tagslam/board2.h"
#include "tagslam/body_defaults.h"
#include "tagslam/yaml_utils.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>

//#define DEBUG_POSE_ESTIMATE
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
      throw std::runtime_error("invalid rigid body type: " + type);
    }
    p->setType(type);
    p->setId(body_id++);
    return (p);
  }

  bool Body::parseCommon(XmlRpc::XmlRpcValue body) {
    bool hasPosePrior = false;
    try {
      double def_pos_noise = BodyDefaults::instance()->positionNoise;
      double def_rot_noise = BodyDefaults::instance()->rotationNoise;
      defaultTagSize_ = static_cast<double>(body["default_tag_size"]);
      isStatic_       = static_cast<bool>(body["is_static"]);
      if (body.hasMember("max_hamming_distance")) {
        maxHammingDistance_ = static_cast<int>(body["max_hamming_distance"]);
      }
      if (body.hasMember("override_tag_position_noise")) {
        overrideTagPositionNoise_ =
          static_cast<double>(body["override_tag_position_noise"]);
      }
      if (body.hasMember("override_tag_rotation_noise")) {
        overrideTagRotationNoise_ =
          static_cast<double>(body["override_tag_rotation_noise"]);
      }
      if (body.hasMember("T_body_odom")) {
        Eigen::Vector3d p =
          yaml_utils::get_vec("position",body["T_body_odom"]["position"]);
        Eigen::Vector3d r =
          yaml_utils::get_vec("rotvec",   body["T_body_odom"]["rotvec"]);
        T_body_odom_ = make_transform(r, p);
      } else {
        T_body_odom_ = Transform::Identity();
      }
      if (body.hasMember("odom_frame_id")) {
        odomFrameId_ = static_cast<std::string>(body["odom_frame_id"]);
      }
      if (body.hasMember("odom_topic")) {
        odomTopic_ = static_cast<std::string>(body["odom_topic"]);
      }
      double odomRotNoise(1e-2), odomTransNoise(5e-2);
      if (body.hasMember("odom_rotation_noise")) {
        odomRotNoise = static_cast<double>(body["odom_rotation_noise"]);
      }
      if (body.hasMember("odom_translation_noise")) {
        odomTransNoise = static_cast<double>(body["odom_translation_noise"]);
      }
      odomNoise_ = PoseNoise2::make(odomRotNoise, odomTransNoise);

      if (body.hasMember("ignore_tags")) {
        auto ignTags = body["ignore_tags"];
        for (const auto i: irange(0, ignTags.size())) {
          ignoreTags_.insert(static_cast<int>(ignTags[i]));
        }
      }
      if (body.hasMember("pose") > 0 && body["pose"].getType() ==
          XmlRpc::XmlRpcValue::TypeStruct) {
        Transform pose;
        PoseNoise2 noise;
        if (yaml_utils::get_pose_and_noise(body["pose"], &pose, &noise,
                                           def_pos_noise, def_rot_noise)) {
          PoseWithNoise pn(pose, noise, true);
          setPoseWithNoise(pn);
          hasPosePrior = true;
        } else {
          setPoseWithNoise(PoseWithNoise());
        }
      }
    } catch (const XmlRpc::XmlRpcException &e) {
      throw std::runtime_error("error parsing body:" + name);
    }
    if (!isStatic_ && hasPosePrior) {
      throw std::runtime_error("dynamic body has prior pose: " + name);
    }
    return (true);
  }

  BodyPtr
  Body::parse_body(const std::string &name,  XmlRpc::XmlRpcValue body) {
    if (!body.hasMember("type")) {
      throw (std::runtime_error("body " + name + " has no type!"));
    }
    std::string type = static_cast<std::string>(body["type"]);
    BodyPtr rb = make_type(name, type);
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
    XmlRpc::XmlRpcValue bodies = config["bodies"];
    if (bodies.getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
      ROS_ERROR_STREAM("no bodies found!");
      throw std::runtime_error("invalid node type for bodies!");
    }
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
