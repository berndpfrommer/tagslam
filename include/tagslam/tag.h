/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef TAGSLAM_TAG_H
#define TAGSLAM_TAG_H

#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
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
    gtsam::Point3 getWorldCorner(int i) const;
    int            id;
    int            type; // distinguishes different size tags!
    double         size;
    gtsam::Pose3   pose; // transforms object to world
    PoseNoise      noise;
    gtsam::Point2  corners[4];
    static std::vector<Tag> parseTags(XmlRpc::XmlRpcValue poses);
  };
}

#endif
