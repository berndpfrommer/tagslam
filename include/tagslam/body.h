/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/pose_with_noise.h"
#include "tagslam/tag.h"
#include <map>
#include <unordered_map>
#include <memory>
#include <iostream>

namespace tagslam {
  class Body {
    using string = std::string;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<Body>            BodyPtr;
    typedef std::shared_ptr<const Body>      BodyConstPtr;
    typedef std::vector<BodyPtr>             BodyVec;
    typedef std::vector<BodyConstPtr>        BodyConstVec;
    typedef std::unordered_map<int, TagPtr>  IdToTagMap;

    // virtual methods to be implemented by derived classes
    virtual bool printTags() const { return (true); }
    virtual bool write(std::ostream &os, const string &prefix) const = 0;
    virtual bool parse(XmlRpc::XmlRpcValue body, const BodyPtr &bp) = 0;
    //

    // getters
    //
    const string &getName() const        { return (name_); }
    const string &getFrameId() const     { return (frameId_); }
    int           getId() const          { return (id_); }
    const string &getOdomTopic() const   { return (odomTopic_); }
    const string &getOdomFrameId() const { return (odomFrameId_); }
    const PoseWithNoise getPoseWithNoise() const  { return (poseWithNoise_); }
    const Transform &getTransformBodyOdom() const { return (T_body_odom_); }
    double getDefaultTagSize() const              { return (defaultTagSize_); }
    const std::list<TagPtr> &getTags() const     { return (tagList_); }
    
    double getOdomTranslationNoise() const {
      return (odomTranslationNoise_); }
    double getOdomRotationNoise() const {
      return (odomRotationNoise_); }
    double getOdomAccelerationNoiseMin() const {
      return (odomAccelerationNoiseMin_); }
    double getOdomAccelerationNoiseMax() const {
      return (odomAccelerationNoiseMax_); }
    double getOdomAngularAccelerationNoiseMin() const {
      return (odomAngularAccelerationNoiseMin_); }
    double getOdomAngularAccelerationNoiseMax() const {
      return (odomAngularAccelerationNoiseMax_); }
    double getFakeOdomTranslationNoise() const {
      return (fakeOdomTranslationNoise_); }
    double getFakeOdomRotationNoise() const {
      return (fakeOdomRotationNoise_); }
    double getOverrideTagRotationNoise() const {
      return (overrideTagRotationNoise_); }
    double getOverrideTagPositionNoise() const {
      return (overrideTagPositionNoise_); }
    bool   isStatic() const { return (isStatic_); }

    // setters

    void   setType(const string &t)                 { type_ = t; }
    void   setId(int id)                            { id_   = id; }
    void   setPoseWithNoise(const PoseWithNoise &p) { poseWithNoise_ = p; }

    // helper functions

    bool   ignoreTag(int tagId) const {
      return (ignoreTags_.count(tagId) != 0); }
    bool    overrides() const {
      return (overrideTagRotationNoise_ > 0 &&
              overrideTagPositionNoise_ > 0);
    }
    TagPtr  findTag(int tagId, int bits) const;
    void    addTag(const TagPtr &tag);
    void    addTags(const TagVec &tags);

    // static functions

    static BodyVec parse_bodies(XmlRpc::XmlRpcValue config);

  protected:
    Body(const string &n  = string(""), bool iS = false) :
      name_(n), frameId_(n), isStatic_(iS) {};
    virtual ~Body() {};
    bool    parseCommon(XmlRpc::XmlRpcValue body);
    bool    writeCommon(std::ostream &os, const string &prefix) const;
    // -------------------------
    string              name_;
    string              frameId_;
    int                 id_{-1};
    bool                isStatic_{true};
    string              type_;
    int                 maxHammingDistance_{2};
    TagMap             tags_; // tags that are hanging off of it
    std::set<int>       ignoreTags_; // reject these tags for this body
    double              defaultTagSize_{0}; // tag size for discovered tags
    PoseWithNoise       poseWithNoise_; // initial pose prior if valid
    double              overrideTagRotationNoise_{-1};
    double              overrideTagPositionNoise_{-1};
    double              fakeOdomTranslationNoise_{-1.0};
    double              fakeOdomRotationNoise_{-1.0};
    // variables used in case odometry data is available
    string              odomTopic_;
    string              odomFrameId_;
    double              odomTranslationNoise_{-1.0};
    double              odomRotationNoise_{-1.0};
    double              odomAccelerationNoiseMin_{5.0}; // m/s^2
    double              odomAngularAccelerationNoiseMin_{5.0}; // rad/sec^2
    double              odomAccelerationNoiseMax_{50.0}; // m/s^2
    double              odomAngularAccelerationNoiseMax_{50.0}; // rad/sec^2
    Transform           T_body_odom_;
    // -------- static functions
    static BodyPtr parse_body(const string &name, XmlRpc::XmlRpcValue config);
  private:
    std::list<TagPtr>   tagList_;
  };
  using BodyPtr      = Body::BodyPtr;
  using BodyConstPtr = Body::BodyConstPtr;
  using BodyVec      = Body::BodyVec;
  using BodyConstVec = Body::BodyConstVec;
}
