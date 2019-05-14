/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/pose_with_noise.h"
#include "tagslam/tag2.h"
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <map>
#include <unordered_map>
#include <memory>
#include <iostream>

namespace tagslam {
  using TagArray = apriltag_msgs::ApriltagArrayStamped;
  using TagArrayConstPtr = TagArray::ConstPtr;
  class Body {
    using string = std::string;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<Body>       BodyPtr;
    typedef std::shared_ptr<const Body> BodyConstPtr;
    typedef std::vector<BodyPtr>        BodyVec;
    typedef std::vector<BodyConstPtr>   BodyConstVec;
    typedef std::unordered_map<int, Tag2Ptr> IdToTagMap;

    // virtual methods to be implemented by derived classes
    virtual bool write(std::ostream &os,
                       const string &prefix) const = 0;
    virtual bool parse(XmlRpc::XmlRpcValue body, const BodyPtr &bp) = 0;
    // public methods
    const string &getName() const { return (name); }
    const string &getFrameId() const { return (frameId_); }
    int           getId() const { return (id_); }
    const string &getOdomTopic() const { return (odomTopic_); }
    const string &getOdomFrameId() const { return (odomFrameId_); }
    double getOdomAcceleration() const { return (odomAcceleration_); }
    double getOdomAngularAcceleration() const { return (odomAngularAcceleration_); }
    const Transform &getTransformBodyOdom() const { return (T_body_odom_); }
    double        getDefaultTagSize() const { return (defaultTagSize_); }
    bool   isStatic() const { return (isStatic_); }
    void   setType(const string &t) { type = t; }
    void   setId(int id) { id_ = id; }
    void   setPoseWithNoise(const PoseWithNoise &p) { poseWithNoise_ = p; }
    const PoseWithNoise getPoseWithNoise() const { return (poseWithNoise_); }
    bool   ignoreTag(int tagId) const { return (ignoreTags_.count(tagId) != 0); }
    Tag2Ptr findTag(int tagId, int bits) const;
    void   addTag(const Tag2Ptr &tag);
    void   addTags(const Tag2Vec &tags);
    bool   overrides() const {
      return (overrideTagRotationNoise_ > 0 &&
              overrideTagPositionNoise_ > 0);
    }
    double getOverrideTagRotationNoise() const {
      return (overrideTagRotationNoise_);
    }
    double getOverrideTagPositionNoise() const {
      return (overrideTagPositionNoise_);
    }
    double getFakeOdomTranslationNoise() const {
      return (fakeOdomTranslationNoise_);
    }
    double getFakeOdomRotationNoise() const {
      return (fakeOdomRotationNoise_);
    }
    std::list<Tag2ConstPtr> getTags() const;
    static BodyVec parse_bodies(XmlRpc::XmlRpcValue config);

  protected:
    Body(const string &n  = string(""), bool iS = false) :
      name(n), frameId_(n), isStatic_(iS) {};
    virtual ~Body() {};
    bool   parseCommon(XmlRpc::XmlRpcValue body);
    bool   writeCommon(std::ostream &os,
                     const string &prefix) const;
    // -------------------------
    string              name;
    string              frameId_;
    int                 id_{-1};
    bool                isStatic_{true};
    string              type;
    int                 maxHammingDistance_{2};
    // tags that are hanging off of it
    Tag2Map             tags;
    // tags that cannot be attached to this body
    std::set<int>       ignoreTags_;
    // default tag size for discovered, unknown tags
    double              defaultTagSize_{0};
    // initial pose prior
    PoseWithNoise       poseWithNoise_;
    // variables used in case odometry data is available
    string              odomTopic_;
    string              odomFrameId_;
    double              odomAcceleration_{5.0}; // m/s^2
    double              odomAngularAcceleration_{5.0}; // rad/sec^2
    Transform           T_body_odom_;
    double              overrideTagRotationNoise_{-1};
    double              overrideTagPositionNoise_{-1};
    double              fakeOdomTranslationNoise_{-1.0};
    double              fakeOdomRotationNoise_{-1.0};
    // -------- static functions
    static BodyPtr parse_body(const string &name,
                              XmlRpc::XmlRpcValue config);
  };
  using BodyPtr = Body::BodyPtr;
  using BodyConstPtr = Body::BodyConstPtr;
  using BodyVec = Body::BodyVec;
  using BodyConstVec = Body::BodyConstVec;
}
