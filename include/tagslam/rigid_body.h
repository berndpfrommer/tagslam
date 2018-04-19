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
    bool   hasTag(int tagId, int bits) const {
      return (findTag(tagId, bits) != NULL); }
    TagPtr findTag(int tagId, int bits) const;
    TagPtr addDefaultTag(int tagId, int bits);
    void   addTag(const TagPtr &tag);
    void   addTags(const TagVec &tags);
    void   attachObservedTag(int cam_idx, const TagPtr &tag);
    unsigned int    attachObservedTags(unsigned int cam_idx,
                                       const TagArrayConstPtr &tags);
    void   detachObservedTags();
    void   getAttachedPoints(int cam_idx,
                             std::vector<gtsam::Point3> *wp,
                             std::vector<gtsam::Point2> *ip,
                             std::vector<int> *tagids = NULL) const;
    int    bestCamera() const;
    // -------------------------
    typedef std::map<int, TagVec> CamToTagVec;
    typedef std::shared_ptr<RigidBody> RigidBodyPtr;
    typedef std::shared_ptr<const RigidBody> RigidBodyConstPtr;
    typedef std::vector<RigidBodyPtr> RigidBodyVec;
    typedef std::vector<RigidBodyConstPtr> RigidBodyConstVec;
    std::string         name;
    std::string         type;
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
  using RigidBodyConstVec = RigidBody::RigidBodyConstVec;
}

#endif
