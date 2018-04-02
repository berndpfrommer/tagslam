/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/rigid_body.h"
#include "tagslam/yaml_utils.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>

namespace tagslam {
  using boost::irange;
  RigidBodyPtr
  RigidBody::parse_body(const std::string &name,
                        XmlRpc::XmlRpcValue rigidBody) {
    RigidBodyPtr body(new RigidBody(name));
    try {
      // first read the header that may contain the body pose
      for (XmlRpc::XmlRpcValue::iterator it = rigidBody.begin();
           it != rigidBody.end(); ++it) {
        if (it->first == "default_tag_size") {
          body->defaultTagSize = static_cast<double>(it->second);
        }
        if (it->first == "is_default_body") {
          body->isDefaultBody = static_cast<bool>(it->second);
        }
        if (it->first == "is_static") {
          body->isStatic = static_cast<bool>(it->second);
        }
        if (it->first == "pose" &&
            it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          gtsam::Pose3 pose;
          PoseNoise noise;
          if (yaml_utils::get_pose_and_noise(it->second, &pose, &noise)) {
            PoseEstimate pe(pose, 0.0, 0, noise);
            body->setPoseEstimate(pe);
          } else {
            body->setPoseEstimate(PoseEstimate()); // invalid
          }
          break;
        }
      }
    } catch (const XmlRpc::XmlRpcException &e) {
      throw std::runtime_error("error parsing header of body:" + name);
    }
    if (body->defaultTagSize == 0) {
      throw std::runtime_error("body " + name + " must have default_tag_size!");
    }
    try {
      for (XmlRpc::XmlRpcValue::iterator it = rigidBody.begin();
           it != rigidBody.end(); ++it) {
        if (it->first == "tags") {
          TagVec tv = Tag::parseTags(it->second, body->defaultTagSize);
          body->addTags(tv);
          break;
        }
      }
    } catch (const XmlRpc::XmlRpcException &e) {
      throw std::runtime_error("error parsing tags of body: " + name);
    }
    return (body);
  }

  TagPtr RigidBody::findTag(int tagId) const {
    const auto it = tags.find(tagId);
    return (it == tags.end() ? NULL: it->second);
  }

  void RigidBody::getAttachedPoints(int cam_idx,
                                    std::vector<gtsam::Point3> *wp,
                                    std::vector<gtsam::Point2> *ip) const {
    const auto tagmap = observedTags.find(cam_idx);
    if (tagmap == observedTags.end()) {
      return; // no tags for this camera
    }
    for (const auto &tag: tagmap->second) {
      if (tag->poseEstimate.isValid()) {
        std::vector<gtsam::Point2> uv = tag->getImageCorners();
        ip->insert(ip->end(), uv.begin(), uv.end());
        const auto opts = tag->getObjectCorners();
        for (const auto &op: opts) {
          wp->push_back(poseEstimate * tag->poseEstimate * op);
        }
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
      TagPtr tagPtr = findTag(tag.id);
      if (tagPtr) { // tag belongs to me
        // transfer corner points to tag
        tagPtr->setImageCorners(&tag.corners[0]);
        attachObservedTag(cam_idx, tagPtr);
        nobs++;
      }
    }
    return (nobs);
  }

  int RigidBody::cameraWithMostAttachedPoints() const {
    int maxCam(-1);
    int maxTags(0);
    for (const auto &cmap: observedTags) {
      if (cmap.second.size() > maxTags) {
        maxCam = cmap.first;
      }
    }
    return (maxCam);
  }

  TagPtr
  RigidBody::addDefaultTag(int tagId) {
    TagPtr tag = Tag::makeTag(tagId, defaultTagSize);
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
  RigidBody::parse_bodies(XmlRpc::XmlRpcValue bodies) {
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
        RigidBodyPtr rb = parse_body(it->first, it->second);
        rb->index = i;
        if (rb->isDefaultBody && foundDefaultBody) {
          throw std::runtime_error("found second default body: " + rb->name);
        }
        rbv.push_back(rb);
        foundDefaultBody = true;
      }
    }
    return (rbv);
  }

}  // namespace
