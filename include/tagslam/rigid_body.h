/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_RIGID_BODY_H
#define TAGSLAM_RIGID_BODY_H

#include "tagslam/tag.h"
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <map>
#include <memory>

namespace tagslam {
  using TagArray = apriltag_msgs::ApriltagArrayStamped;
  using TagArrayConstPtr = TagArray::ConstPtr;
  struct RigidBody {
    RigidBody(const std::string &n  = std::string(""),
              const gtsam::Pose3 &p = gtsam::Pose3(),
              const Tag::PoseNoise &pn = Tag::PoseNoise(),
              bool iS = false) :
      name(n), pose(p), noise(pn), isStatic(iS) {};
    bool   hasTag(int tagId) const { return (findTag(tagId) != NULL); }
    TagPtr findTag(int tagId) const {
      const auto it = tags.find(tagId);
      return (it == tags.end() ? NULL: it->second);
    }
    TagPtr addDefaultTag(int tagId);
    void   addTag(const TagPtr &tag);
    void   addTags(const TagVec &tags);
    void   attachObservedTag(int cam_idx, const TagPtr &tag);
    void   attachObservedTags(unsigned int cam_idx,
                              const TagArrayConstPtr &tags);
    void   detachObservedTags();
    void   getAttachedPoints(int cam_idx,
                             std::vector<gtsam::Point3> *wp,
                             std::vector<gtsam::Point2> *ip) const;
    int    cameraWithMostAttachedPoints() const;
    // -------------------------
    typedef std::map<int, TagVec> CamToTagVec;
    typedef std::shared_ptr<RigidBody> RigidBodyPtr;
    typedef std::shared_ptr<const RigidBody> RigidBodyConstPtr;
    typedef std::vector<RigidBodyPtr> RigidBodyVec;
    std::string         name;
    int                 index{-1};
    gtsam::Pose3        pose;
    Tag::PoseNoise      noise;
    bool                isStatic;
    bool                isDefaultBody;
    bool                hasValidPoseEstimate{false};
    TagMap              tags;
    CamToTagVec         observedTags;
    double              defaultSize{0};
    // -------- static functions
    static RigidBodyPtr parse_body(const std::string &name,
                                   XmlRpc::XmlRpcValue body);
    static RigidBodyVec parse_bodies(XmlRpc::XmlRpcValue bodies);
  };
  using RigidBodyPtr = RigidBody::RigidBodyPtr;
  using RigidBodyConstPtr = RigidBody::RigidBodyConstPtr;
  using RigidBodyVec = RigidBody::RigidBodyVec;
}

#endif
