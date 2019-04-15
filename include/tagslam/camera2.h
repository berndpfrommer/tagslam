/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/camera_intrinsics2.h"
#include "tagslam/pose_with_noise.h"
#include <ros/ros.h>
#include <memory>
#include <map>
#include <set>
#include <string>

namespace tagslam {
  class Body; // need forward declaration here
  class Camera2 {
    using string = std::string;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // --------- typedefs -------
    typedef std::shared_ptr<Camera2>       Camera2Ptr;
    typedef std::shared_ptr<const Camera2> Camera2ConstPtr;
    typedef std::vector<Camera2Ptr>        Camera2Vec;
    // --- getters/setters
    const string &getName()       const { return (name_); }
    const string &getImageTopic() const { return (imageTopic_); }
    const string &getTagTopic()   const { return (tagTopic_); }
    const string &getFrameId()    const { return (frameId_); }
    const string &getRigName()    const { return (rigName_); }
    const std::shared_ptr<Body> getRig() const { return (rig_); }
    const PoseNoise2 &getWiggle() const { return (wiggle_); }
    const CameraIntrinsics2& getIntrinsics() const { return (intrinsics_); }
    void setRig(const std::shared_ptr<Body> &rig) { rig_ = rig; }
    
    // --- static methods
    static Camera2 parse_camera(const string &prefix, const string &name,
                                const ros::NodeHandle &nh);
    static Camera2Vec parse_cameras(const string &prefix,
                                    const ros::NodeHandle &nh);
  private:
    // -------- variables -------
    string                name_;
    int                   index_{-1};
    string                imageTopic_; // topic for images
    string                tagTopic_;   // topic for tags
    string                frameId_;    // ros frame id of camera
    string                rigName_;    // name of rig body
    std::shared_ptr<Body> rig_;        // pointer to rig body
    CameraIntrinsics2     intrinsics_; // intrinsic calibration
    PoseNoise2            wiggle_;     // how rigid the ext calib is
  };

  using Camera2Ptr      = Camera2::Camera2Ptr;
  using Camera2ConstPtr = Camera2::Camera2ConstPtr;
  using Camera2Vec      = Camera2::Camera2Vec;
}
