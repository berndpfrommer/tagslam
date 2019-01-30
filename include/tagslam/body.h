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
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Body(const std::string &n  = std::string(""), bool iS = false) :
      name(n), isStatic(iS) {};
    virtual ~Body() {};

    typedef std::shared_ptr<Body>       BodyPtr;
    typedef std::shared_ptr<const Body> BodyConstPtr;
    typedef std::vector<BodyPtr>        BodyVec;
    typedef std::vector<BodyConstPtr>   BodyConstVec;
    typedef std::unordered_map<int, Tag2Ptr> IdToTagMap;

    // virtual methods to be implemented by derived classes
    virtual bool write(std::ostream &os,
                       const std::string &prefix) const = 0;
    virtual bool parse(XmlRpc::XmlRpcValue body) = 0;
    // public methods
    const std::string &getName() const { return(name); }
    bool   getIsStatic() const { return (isStatic); }
    void   setType(const std::string &t) { type = t; }
    void   setPoseWithNoise(const PoseWithNoise &p) { poseWithNoise = p; }
    const PoseWithNoise getPoseWithNoise() const { return (poseWithNoise); }
 
    Tag2Ptr findTag(int tagId, int bits) const;
    void   addTag(const Tag2Ptr &tag);
    void   addTags(const Tag2Vec &tags);
    std::list<Tag2ConstPtr> getTags() const;
    static BodyVec parse_bodies(XmlRpc::XmlRpcValue config);

  protected:
    bool   parseCommon(XmlRpc::XmlRpcValue body);
    bool   writeCommon(std::ostream &os,
                     const std::string &prefix) const;
    // -------------------------
    std::string         name;
    bool                isStatic{true};
    std::string         type;
    //
    int                 maxHammingDistance{2};
    // tags that are hanging off of it
    Tag2Map             tags;
    // tags that cannot be attached to this body
    std::set<int>       ignoreTags;
    // default tag size for discovered, unknown tags
    double              defaultTagSize{0};
    // any initial pose prior
    PoseWithNoise       poseWithNoise;
    // variables used in case odometry data is available
    std::string         odomFrameId;
    PoseNoise2          odomNoise;
    Transform           T_body_odom;
    // -------- static functions
    static BodyPtr parse_body(const std::string &name,
                              XmlRpc::XmlRpcValue config);
  };
  using BodyPtr = Body::BodyPtr;
  using BodyConstPtr = Body::BodyConstPtr;
  using BodyVec = Body::BodyVec;
  using BodyConstVec = Body::BodyConstVec;
}
