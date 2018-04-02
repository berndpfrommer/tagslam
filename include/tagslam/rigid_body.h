/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_RIGID_BODY_H
#define TAGSLAM_RIGID_BODY_H

#include "tagslam/pose_estimate.h"
#include "tagslam/tag.h"
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <map>
#include <memory>

namespace tagslam {
  using TagArray = apriltag_msgs::ApriltagArrayStamped;
  using TagArrayConstPtr = TagArray::ConstPtr;
  struct RigidBody {
    RigidBody(const std::string &n  = std::string(""),
              bool iS = false) :
      name(n), isStatic(iS) {};

    void   setPoseEstimate(const PoseEstimate &pe) { poseEstimate = pe; }
    void   setIsDefaultBody(bool b) { isDefaultBody = b; }
    bool   hasTag(int tagId) const { return (findTag(tagId) != NULL); }
    TagPtr findTag(int tagId) const;
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
    PoseEstimate        poseEstimate;
    bool                isStatic{true};
    bool                isDefaultBody{false};
    TagMap              tags;
    CamToTagVec         observedTags;
    double              defaultTagSize{0};
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
