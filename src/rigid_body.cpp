/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/rigid_body.h"
#include "tagslam/simple_body.h"
#include "tagslam/board.h"
#include "tagslam/yaml_utils.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>

//#define DEBUG_POSE_ESTIMATE
namespace tagslam {
  using boost::irange;

  static RigidBodyPtr make_type(const std::string &name,
                                const std::string &type) {
    RigidBodyPtr p;
    if (type == "board") {
      BoardPtr board(new Board(name));
      p = board;
    } else  if (type == "simple") {
      SimpleBodyPtr sb(new SimpleBody(name));
      p = sb;
    } else if (type == "camera_rig") {
      SimpleBodyPtr sb(new SimpleBody(name));
      p = sb;
    } else {
      throw std::runtime_error("invalid rigid body type: " + type);
    }
    p->type = type;
    return (p);
  }

  bool RigidBody::parseCommon(XmlRpc::XmlRpcValue bodyDefaults,
                              XmlRpc::XmlRpcValue body) {
    try {
      double def_pos_noise = static_cast<double>(bodyDefaults["position_noise"]);
      double def_rot_noise = static_cast<double>(bodyDefaults["rotation_noise"]);
      defaultTagSize = static_cast<double>(body["default_tag_size"]);
      isDefaultBody  = static_cast<bool>(body["is_default_body"]);
      isStatic       = static_cast<bool>(body["is_static"]);
      if (body.hasMember("max_hamming_distance")) {
        maxHammingDistance = static_cast<int>(body["max_hamming_distance"]);
      }
      if (body.hasMember("T_body_odom")) {
        Eigen::Vector3d p = yaml_utils::get_vec("position", body["T_body_odom"]["position"]);
        Eigen::Vector3d r = yaml_utils::get_vec("rotvec",   body["T_body_odom"]["rotvec"]);
        T_body_odom = gtsam::Pose3(gtsam::Rot3(utils::rotmat(r)), gtsam::Point3(p));
      }
      if (body.hasMember("odom_frame_id")) {
        odomFrameId = static_cast<std::string>(body["odom_frame_id"]);
      }
      double odomRotNoise(1e-2), odomTransNoise(5e-2);
      if (body.hasMember("odom_rotation_noise")) {
        odomRotNoise = static_cast<double>(body["odom_rotation_noise"]);
      }
      if (body.hasMember("odom_translation_noise")) {
        odomTransNoise = static_cast<double>(body["odom_translation_noise"]);
      }
      odomNoise = makePoseNoise(odomRotNoise, odomTransNoise);

      if (body.hasMember("ignore_tags")) {
        auto ignTags = body["ignore_tags"];
        for (const auto i: irange(0, ignTags.size())) {
          ignoreTags.insert(static_cast<int>(ignTags[i]));
        }
      }
      if (body.hasMember("pose") > 0 && body["pose"].getType() ==
          XmlRpc::XmlRpcValue::TypeStruct) {
          gtsam::Pose3 pose;
          PoseNoise noise;
          if (yaml_utils::get_pose_and_noise(body["pose"], &pose, &noise,
                                             def_pos_noise, def_rot_noise)) {
            PoseEstimate pe(pose, 0.0, 0, noise);
            setPoseEstimate(pe);
            hasPosePrior = true;
          } else {
            setPoseEstimate(PoseEstimate()); // invalid
          }
      }
    } catch (const XmlRpc::XmlRpcException &e) {
      throw std::runtime_error("error parsing body:" + name);
    }
    if (!isStatic && hasPosePrior) {
      throw std::runtime_error("dynamic body has prior pose: " + name);
    }
    return (true);
  }

  RigidBodyPtr
  RigidBody::parse_body(const std::string &name,
                        XmlRpc::XmlRpcValue bodyDefaults,
                        XmlRpc::XmlRpcValue body) {
    if (!body.hasMember("type")) {
      throw (std::runtime_error("rigid body " + name + " has no type!"));
    }
    std::string type = static_cast<std::string>(body["type"]);
    RigidBodyPtr rb = make_type(name, type);
    rb->parse(bodyDefaults, body);
    if (rb->isDefaultBody && rb->defaultTagSize == 0) {
      throw std::runtime_error("default body " + name + " must have default_tag_size!");
    }
    return (rb);
  }

  TagPtr RigidBody::findTag(int tagId, int bits) const {
    const auto it = tags.find(tagId);
    return ((it == tags.end() || it->second->bits != bits)? NULL: it->second);
  }

  void RigidBody::updateAttachedTagPoses(const IdToTagMap &updatedTags) {
    for (const auto &camToTags: observedTags) {
      for (auto &tag: camToTags.second) {
        IdToTagMap::const_iterator it = updatedTags.find(tag->id);
        if (it == updatedTags.end()) {
          std::cout << "ERROR: unknown tag: " << tag->id;
          continue;
        }
        if (it->second->poseEstimate.isValid()) {
#ifdef DEBUG_POSE_ESTIMATE          
          std::cout << "observed tag pose updated for tag: " << it->first << std::endl;
#endif          
          tag->poseEstimate = it->second->poseEstimate;
        }
      }
    }
  }

  void RigidBody::getAttachedPoints(int cam_idx,
                                    std::vector<gtsam::Point3> *wp,
                                    std::vector<gtsam::Point2> *ip,
                                    std::vector<gtsam::Pose3>  *T_w_o,
                                    bool pointsInWorldCoordinates,
                                    std::vector<int> *tagids) const {
    const auto tagmap = observedTags.find(cam_idx);
    if (tagmap == observedTags.end()) {
      return;
    }
    // return points in body coordinates or world coordinates,
    // depending on what is asked for
    gtsam::Pose3 T_w_b = pointsInWorldCoordinates ? poseEstimate.getPose() : gtsam::Pose3();
#ifdef DEBUG_POSE_ESTIMATE    
    std::cout << name << " get attached points for cam " << cam_idx << ", T_w_b: " << std::endl << T_w_b << std::endl;
#endif    
    for (const auto &tag: tagmap->second) {
      if (tag->poseEstimate.isValid()) {
        if (tagids) tagids->push_back(tag->id);
        std::vector<gtsam::Point2> uv = tag->getImageCorners();
        ip->insert(ip->end(), uv.begin(), uv.end());
#ifdef DEBUG_POSE_ESTIMATE        
        std::cout << "tag pose for tag: " << tag->id << std::endl << tag->poseEstimate << std::endl;
#endif        
        const auto opts = tag->getObjectCorners();
        T_w_o->push_back(T_w_b * tag->poseEstimate);
        for (const auto &op: opts) {
          wp->push_back(T_w_o->back() * op);
          //std::cout << "tag id: " << tag->id << " wp: " << wp->back() << std::endl;
        }
      } else {
#ifdef DEBUG_POSE_ESTIMATE
        std::cout << "INVALID tag pose for tag: " << tag->id << std::endl;
#endif
      }
    }
  }

  void RigidBody::detachObservedTags() {
    observedTags.clear();
  }

  void
  RigidBody::attachObservedTag(int cam_idx, const TagPtr &tagPtr) {
    // attach as observed
    if (observedTags.count(cam_idx) == 0) {
      // need to insert a new cam->tagmap entry
      observedTags.insert(CamToTagVec::value_type(cam_idx, TagVec()));
    }
    TagVec &tvec = observedTags.find(cam_idx)->second;
    tvec.push_back(tagPtr);
  }

  unsigned int
  RigidBody::attachObservedTags(unsigned int cam_idx,
                                const TagArrayConstPtr &tags) {
    unsigned int nobs(0);
    for (const auto &tag: tags->apriltags) {
      if (tag.hamming > maxHammingDistance) {
        ROS_INFO_STREAM(name << " IGNORING tag " << tag.id << " with hamming dist: " << tag.hamming);
        continue;
      }
      if (ignoreTags.find(tag.id) != ignoreTags.end()) {
        ROS_INFO_STREAM("IGNORING disallowed tag " << tag.id);
        continue;
      }
      TagPtr tagPtr = findTag(tag.id, tag.bits);
      if (tagPtr) { // tag belongs to me
        TagPtr newTag(new Tag(*tagPtr));
        // transfer corner points to tag
        newTag->setImageCorners(&tag.corners[0]);
        attachObservedTag(cam_idx, newTag);
        nobs++;
      }
    }
    return (nobs);
  }

  int RigidBody::bestCamera(const std::vector<int> &cams) const {
    int maxCam(-1);
    double maxVariance(0.0);
    for (const auto &cam_idx: cams) {
      const auto it = observedTags.find(cam_idx);
      if (it == observedTags.end()) {
        continue; // no tags seen for this cameraa
      }
      const auto &cmap = *it;
      gtsam::Point2 sum(0.0, 0.0);
      gtsam::Point2 sumsq(0.0, 0.0);
      int cnt(0);
      for (const auto &tag: cmap.second) {
        const auto &uv = tag->getImageCorners();
        for (const auto i: irange(0,4)) {
          sum   += uv[i];
          sumsq += gtsam::Point2(uv[i].x() * uv[i].x(),
                                 uv[i].y() * uv[i].y());
          cnt++;
        }
      }
      if (cnt > 0) {
        gtsam::Point2 meansq((sum.x()/cnt) * (sum.x()/cnt),
                             (sum.y()/cnt) * (sum.y()/cnt));
        gtsam::Point2 var = sumsq/cnt - meansq;
        double v = var.x() + var.y();
        //std::cout << "cam: " << cmap.first << " has sigma: " << std::sqrt(v) << std::endl;
        if (v > maxVariance) {
          maxVariance = v;
          maxCam = cmap.first;
        }
      }
    }
    return (maxCam);
  }

  TagPtr
  RigidBody::addDefaultTag(int tagId, int bits) {
    TagPtr tag = Tag::makeTag(tagId, bits, defaultTagSize);
    addTag(tag);
    return (tag);
  }

  void RigidBody::addTag(const TagPtr &tag) {
    tags.insert(TagMap::value_type(tag->id, tag));
  }

  void RigidBody::addTags(const TagVec &tags) {
    for (const auto &tag:tags) {
      addTag(tag);
    }
  }
  
  RigidBodyVec
  RigidBody::parse_bodies(XmlRpc::XmlRpcValue body_defaults,
                          XmlRpc::XmlRpcValue bodies) {
    RigidBodyVec rbv;
    bool foundDefaultBody{false};
    if (bodies.getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
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
        RigidBodyPtr rb = parse_body(it->first, body_defaults, it->second);
        rb->index = i;
        if (rb->isDefaultBody && foundDefaultBody) {
          throw std::runtime_error("found second default body: " + rb->name);
        }
        rbv.push_back(rb);
        foundDefaultBody = rb->isDefaultBody;
      }
    }
    return (rbv);
  }

  bool RigidBody::writeCommon(std::ostream &os, const std::string &prefix) const {
    os << prefix << "- " << name << ":" << std::endl;
    std::string pfix = prefix + "    ";
    os << pfix << "type: " << type << std::endl;
    os << pfix << "is_default_body: " <<
      (isDefaultBody ? "true" : "false") << std::endl;
    os << pfix << "is_static: " <<
      (isStatic ? "true" : "false") << std::endl;
    os << pfix << "default_tag_size: " << defaultTagSize << std::endl;
    if (isStatic) {
      os << pfix << "pose:" << std::endl;;
      PoseNoise smallNoise = makePoseNoise(0.001, 0.001);
      yaml_utils::write_pose(os, pfix + "  ", poseEstimate, smallNoise,
                             hasPosePrior);
      // TODO: poseEstimate.getNoise());
    }
    return (true);
  }

}  // namespace
