/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_TAG_H
#define TAGSLAM_TAG_H

#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <vector>
#include <map>
#include <memory>

namespace tagslam {
  struct Tag {
    typedef gtsam::noiseModel::Diagonal::shared_ptr   PoseNoise;
    Tag(int ida, int tp, double sz,
        const gtsam::Pose3 &p,  const PoseNoise &pn, bool hasKPose);

    gtsam::Point3 getObjectCorner(int i) const;
    const std::vector<gtsam::Point3> &getObjectCorners() const {
      return objectCorners;
    };
    const std::vector<gtsam::Point2> &getImageCorners() const {
      return imageCorners;
    }
    gtsam::Point3 getWorldCorner(int i) const;
    bool hasValidImageCorners() const;
    void setImageCorners(const geometry_msgs::Point *corners);

    // ------- variables --------------
    int            id;
    int            type;  // distinguishes different size tags!
    double         size;  // tag size in meters
    gtsam::Pose3   pose;  // tag pose relative to rigid body: T_s_o
    PoseNoise      noise; // known noise of pose
    bool           hasKnownPose; // tag pose is known from the start
    bool           hasEstimatedPose{false}; // tag pose has valid estimate
    //
    typedef std::shared_ptr<Tag>        TagPtr;
    typedef std::shared_ptr<const Tag>  TagConstPtr;
    typedef std::vector<TagPtr>         TagVec;
    // ----------- static methods
    static std::vector<gtsam::Point3> make_object_corners(double size);
    static TagPtr makeTag(int ida, double sz,
                          const gtsam::Pose3 &p = gtsam::Pose3(),
                          const PoseNoise &pn = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector(6)),
                          bool hasKPose = false);

    static TagVec parseTags(XmlRpc::XmlRpcValue xmltags, double sz);
  private:
    std::vector<gtsam::Point3>  objectCorners;  // 3d object coordinates
    std::vector<gtsam::Point2>  imageCorners;   // u,v of last observed corner points
  };
  typedef Tag::TagPtr TagPtr;
  typedef Tag::TagConstPtr TagConstPtr;
  typedef Tag::TagVec TagVec;
  typedef std::map<int, TagPtr> TagMap;
}

#endif
