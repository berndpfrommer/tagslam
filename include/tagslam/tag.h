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

namespace tagslam {
  struct Tag {
    typedef gtsam::noiseModel::Diagonal::shared_ptr   PoseNoise;
    Tag(int ida = 0, int tp = 0, double sz = 0,
        const gtsam::Pose3 &p = gtsam::Pose3(),
        PoseNoise pn = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector(6))):
      id(ida), type(tp), size(sz), pose(p), noise(pn) { };
    gtsam::Point3 getObjectCorner(int i) const;
    std::vector<gtsam::Point3> getObjectCorners() const;
    std::vector<gtsam::Point2> getImageCorners() const;
    gtsam::Point3 getWorldCorner(int i) const;
    bool hasValidImageCorners() const;
    void setCorners(const geometry_msgs::Point *corners);

    static std::vector<gtsam::Point3> get_object_corners(double size);
    // ------- variables --------------
    int            id;
    int            type; // distinguishes different size tags!
    double         size;
    gtsam::Pose3   pose; // tag pose relative to parent: T_s_o
    PoseNoise      noise;
    gtsam::Point2  corners[4];  // u,v of last observed corner points
    unsigned int   parentIdx{0}; // parent object idx
    static std::vector<Tag> parseTags(XmlRpc::XmlRpcValue poses);
  };
}

#endif
