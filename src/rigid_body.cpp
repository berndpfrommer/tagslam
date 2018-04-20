/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/rigid_body.h"
#include "tagslam/yaml_utils.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>

namespace tagslam {
  using boost::irange;
  static void parse_type(const RigidBodyPtr &rb,
                         const std::string &type,
                         XmlRpc::XmlRpcValue xml) {
    if (type != "board") {
      throw std::runtime_error("only board type supported!");
    }
    int tag_start_id = -1;
    double tag_size = -1.0;
    int tag_bits = 6;
    double tag_spacing = 0.25;
    int tag_rows = -1;
    int tag_columns = -1;
    double tag_rot_noise = 0.001;
    double tag_pos_noise = 0.001;
    try {
      for (XmlRpc::XmlRpcValue::iterator it = xml.begin();
           it != xml.end(); ++it) {
        const auto &key = it->first;
        auto &val = it->second;
        if (key == "tag_start_id") tag_start_id = static_cast<int>(val);
        if (key == "tag_size")     tag_size = static_cast<double>(val);
        if (key == "tag_bits")     tag_bits = static_cast<int>(val);
        if (key == "tag_spacing")  tag_spacing = static_cast<double>(val);
        if (key == "tag_rows")     tag_rows = static_cast<int>(val);
        if (key == "tag_columns")  tag_columns = static_cast<int>(val);
        if (key == "tag_rotation_noise")  tag_rot_noise = static_cast<double>(val);
        if (key == "tag_position_noise")  tag_pos_noise = static_cast<double>(val);
      }
    } catch (const XmlRpc::XmlRpcException &e) {
      throw std::runtime_error("error parsing type:" + type + " of body: " + rb->name);
    }
    if (tag_rows < 0 || tag_columns < 0) {
      throw std::runtime_error("must specify tag rows and cols for " + type + " of body: " + rb->name);
    }
    if (tag_start_id < 0 || tag_size < 0) {
      throw std::runtime_error("must specify tag start id and size for " + type + " of body: " + rb->name);
    }
    int tagid = tag_start_id;
    for (int row = 0; row < tag_rows; row++) {
      for (int col = 0; col < tag_columns; col++) {
        gtsam::Pose3 pose(gtsam::Rot3(),
                          gtsam::Point3(col * tag_size * (1.0 + tag_spacing),
                                        row * tag_size * (1.0 + tag_spacing), 0.0));
        PoseNoise noise = makePoseNoise(tag_rot_noise, tag_pos_noise);
        PoseEstimate pe(pose, 0.0, 0, noise);
        rb->addTag(Tag::makeTag(tagid++, tag_bits, tag_size, pe, true));
      }
    }
  }

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
        if (it->first == "type") {
          body->type = static_cast<std::string>(it->second);
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
        if (it->first == body->type) {
          parse_type(body, body->type, it->second);
        }
      }
    } catch (const XmlRpc::XmlRpcException &e) {
      throw std::runtime_error("error parsing tags of body: " + name);
    }
    return (body);
  }

  TagPtr RigidBody::findTag(int tagId, int bits) const {
    const auto it = tags.find(tagId);
    return ((it == tags.end() || it->second->bits != bits)? NULL: it->second);
  }

  void RigidBody::getAttachedPoints(int cam_idx,
                                    std::vector<gtsam::Point3> *wp,
                                    std::vector<gtsam::Point2> *ip,
                                    std::vector<int> *tagids) const {
    const auto tagmap = observedTags.find(cam_idx);
    if (tagmap == observedTags.end()) {
      return;
    }
    //std::cout << name << " get attached points pose estimate: " << poseEstimate << std::endl;
    for (const auto &tag: tagmap->second) {
      if (tag->poseEstimate.isValid()) {
        if (tagids) tagids->push_back(tag->id);
        std::vector<gtsam::Point2> uv = tag->getImageCorners();
        ip->insert(ip->end(), uv.begin(), uv.end());
        //std::cout << "tag pose: " << tag->poseEstimate << std::endl;
        const auto opts = tag->getObjectCorners();
        for (const auto &op: opts) {
          wp->push_back(poseEstimate * tag->poseEstimate * op);
          //std::cout << "tag id: " << tag->id << " wp: " << wp->back() << std::endl;
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

  int RigidBody::bestCamera() const {
    int maxCam(-1);
    double maxVariance(0.0);
    for (const auto &cmap: observedTags) { // loop over cameras
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
        std::cout << "cam: " << cmap.first << " has sigma: " << std::sqrt(v) << std::endl;
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
        foundDefaultBody = rb->isDefaultBody;
      }
    }
    return (rbv);
  }

}  // namespace
