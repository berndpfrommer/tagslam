/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_RIGID_BODY_H
#define TAGSLAM_RIGID_BODY_H

#include "tagslam/pose_estimate.h"
#include "tagslam/tag.h"
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <map>
#include <unordered_map>
#include <memory>
#include <iostream>

namespace tagslam {
  using TagArray = apriltag_msgs::ApriltagArrayStamped;
  using TagArrayConstPtr = TagArray::ConstPtr;
  struct RigidBody {
    RigidBody(const std::string &n  = std::string(""),
              bool iS = false) :
      name(n), isStatic(iS) {};
    virtual ~RigidBody() {};
    
    typedef std::shared_ptr<RigidBody>       RigidBodyPtr;
    typedef std::shared_ptr<const RigidBody> RigidBodyConstPtr;
    typedef std::vector<RigidBodyPtr>        RigidBodyVec;
    typedef std::vector<RigidBodyConstPtr>   RigidBodyConstVec;
    typedef std::unordered_map<int, TagPtr>       IdToTagMap;

    virtual bool write(std::ostream &os, const std::string &prefix) const = 0;
    virtual bool parse(XmlRpc::XmlRpcValue body_defaults,
                       XmlRpc::XmlRpcValue body) = 0;
    bool parseCommon(XmlRpc::XmlRpcValue bodyDefaults,
                     XmlRpc::XmlRpcValue body);
    bool writeCommon(std::ostream &os, const std::string &prefix) const;
    
    void   setPoseEstimate(const PoseEstimate &pe) { poseEstimate = pe; }
    void   setIsDefaultBody(bool b) { isDefaultBody = b; }
    bool   hasTag(int tagId, int bits) const {
      return (findTag(tagId, bits) != NULL); }
    bool   isCameraRig() const { return (type == "camera_rig"); }
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
                             std::vector<gtsam::Pose3>  *T_w_o,
                             bool inWorldCoordinates,
                             std::vector<int> *tagids = NULL) const;
    int bestCamera(const std::vector<int> &cams) const;
    // -------------------------
    typedef std::map<int, TagVec> CamToTagVec;
    std::string         name;
    std::string         type;
    std::string         odomFrameId;
    PoseNoise           odomNoise;
    int                 index{-1};
    PoseEstimate        poseEstimate;
    bool                isStatic{true};
    bool                isDefaultBody{false};
    int                 maxHammingDistance{2};
    TagMap              tags;
    CamToTagVec         observedTags;
    double              defaultTagSize{0};
    bool                hasPosePrior{false};
    std::set<int>       ignoreTags;
    gtsam::Pose3        T_body_odom;
    // -------- static functions
    static RigidBodyPtr parse_body(const std::string &name,
                                   XmlRpc::XmlRpcValue bodyDefaults,
                                   XmlRpc::XmlRpcValue body);
    void updateAttachedTagPoses(const IdToTagMap &updatedTags);
    static RigidBodyVec parse_bodies(XmlRpc::XmlRpcValue body_defaults,
                                     XmlRpc::XmlRpcValue bodies);
  };
  using RigidBodyPtr = RigidBody::RigidBodyPtr;
  using RigidBodyConstPtr = RigidBody::RigidBodyConstPtr;
  using RigidBodyVec = RigidBody::RigidBodyVec;
  using RigidBodyConstVec = RigidBody::RigidBodyConstVec;
}

#endif
