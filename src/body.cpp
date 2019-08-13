/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/body.h"
#include "tagslam/logging.h"
#include "tagslam/simple_body.h"
#include "tagslam/board.h"
#include "tagslam/body_defaults.h"
#include "tagslam/yaml_utils.h"
#include "tagslam/xml.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>


namespace tagslam {
  using boost::irange;

  static int body_id = 0;

  static BodyPtr make_type(const std::string &name, const std::string &type) {
    BodyPtr p;
    if (type == "board") {
      BoardPtr board(new Board(name));
      p = board;
    } else  if (type == "simple") {
      SimpleBodyPtr sb(new SimpleBody(name));
      p = sb;
    } else {
      if (type == "camera_rig") {
        ROS_ERROR_STREAM("camera_rig is deprecated, change to simple!");
      }
      BOMB_OUT("invalid rigid body type: " + type);
    }
    p->setType(type);
    p->setId(body_id++);
    return (p);
  }

  bool Body::parseCommon(XmlRpc::XmlRpcValue body) {
    try {
      isStatic_       = xml::parse<bool>(body,  "is_static");
      defaultTagSize_ = xml::parse<double>(body, "default_tag_size", 0.0);
      maxHammingDistance_ = xml::parse<int>(body, "max_hamming_distance", 2);
      overrideTagPositionNoise_ =
        xml::parse<double>(body, "override_tag_position_noise", -1.0);
      overrideTagRotationNoise_ =
        xml::parse<double>(body, "override_tag_rotation_noise", -1.0);
      T_body_odom_      = xml::parse<Transform>(body,  "T_body_odom",
                                                Transform::Identity());
      odomFrameId_      = xml::parse<string>(body, "odom_frame_id", "");
      odomTopic_        = xml::parse<string>(body, "odom_topic", "");
      odomTranslationNoise_ = xml::parse<double>(
        body, "odom_translation_noise", -1.0);
      odomRotationNoise_ = xml::parse<double>(
        body, "odom_rotation_noise", -1.0);
      // first read old tag, then new one if provided
      const double oanm = xml::parse<double>(body, "odom_acceleration", 5.0);
      odomAccelerationNoiseMin_ =
        xml::parse<double>(body, "odom_acceleration_noise_min", oanm);
      
      const double oaanm = xml::parse<double>(body, "odom_angular_acceleration",5.0);
      odomAngularAccelerationNoiseMin_=
        xml::parse<double>(body, "odom_angular_acceleration_noise_min", oaanm);
      
      odomAccelerationNoiseMax_ =
        xml::parse<double>(body, "odom_acceleration_noise_max",
                           10 * odomAccelerationNoiseMin_);
      odomAngularAccelerationNoiseMax_ =
        xml::parse<double>(body, "odom_angular_acceleration_noise_max",
                           10 * odomAngularAccelerationNoiseMin_);
      
      ignoreTags_ = xml::parse_container<std::set<int>>(body, "ignore_tags",
                                                        std::set<int>());
      poseWithNoise_ = xml::parse<PoseWithNoise>(body, "pose",
                                                 PoseWithNoise());
      if (poseWithNoise_.isValid() && !isStatic_) {
        BOMB_OUT("body " << getName() << " is dynamic but has pose!");
      }
      fakeOdomTranslationNoise_ =
        xml::parse<double>(body, "fake_odom_translation_noise", -1.0);
      fakeOdomRotationNoise_ =
        xml::parse<double>(body, "fake_odom_rotation_noise", -1.0);
    } catch (const XmlRpc::XmlRpcException &e) {
      BOMB_OUT("error parsing body: " << name_);
    }
    if (!isStatic_ && poseWithNoise_.isValid()) {
      BOMB_OUT("dynamic body has prior pose: " + name_);
    }
    return (true);
  }

  BodyPtr
  Body::parse_body(const string &name,  XmlRpc::XmlRpcValue body) {
    const string type = xml::parse<string>(body, "type");
    const BodyPtr rb = make_type(name, type);
    rb->parseCommon(body);
    rb->parse(body, rb);
    return (rb);
  }

  TagPtr Body::findTag(int tagId, int bits) const {
    const auto it = tags_.find(tagId);
    return ((it == tags_.end() || it->second->getBits() != bits)?
            NULL: it->second);
  }

  void Body::addTag(const TagPtr &tag) {
    tags_.insert(TagMap::value_type(tag->getId(), tag));
    tagList_.push_back(tag);
  }

  void Body::addTags(const TagVec &tags) {
    for (const auto &tag:tags) {
      addTag(tag);
    }
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

  bool Body::writeCommon(std::ostream &os, const string &prefix) const {
    os << prefix << "- " << name_ << ":" << std::endl;
    string pfix = prefix + "    ";
    os << pfix << "type: " << type_ << std::endl;
    os << pfix << "is_static: " <<
      (isStatic_ ? "true" : "false") << std::endl;
    os << pfix << "default_tag_size: " << defaultTagSize_ << std::endl;
    return (true);
  }

}  // namespace
