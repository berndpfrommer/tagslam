/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/pose_with_noise.h"
#include "tagslam/geometry.h"

#include <vector>
#include <map>
#include <memory>
#include <iostream>
#include <ros/ros.h>

namespace tagslam {
  class Body;
  class Tag {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //
    typedef std::shared_ptr<Tag>        TagPtr;
    typedef std::shared_ptr<const Tag>  TagConstPtr;
    typedef std::vector<TagPtr>         TagVec;

    int    getId()   const { return (id_); }
    int    getBits() const { return (bits_); }
    double getSize() const { return (size_); }
    const std::shared_ptr<Body> getBody() const { return (body_); }
    const PoseWithNoise &getPoseWithNoise() const { return (poseWithNoise_); }
    const Eigen::Matrix<double, 4, 3> &getObjectCorners()
      const { return (objectCorners_); }
    const Point3d getObjectCorner(int idx)
      const { return (idx >= 0 ? objectCorners_.row(idx) : Point3d(0,0,0)); }

    friend std::ostream &operator<<(std::ostream &os, const Tag &tag);
    // ----------- static methods
    static TagVec parseTags(XmlRpc::XmlRpcValue xmltags, double sz,
                             const std::shared_ptr<Body> &body);
    static TagPtr make(int ida, int bits, double sz, const PoseWithNoise &pe,
                        const std::shared_ptr<Body> &body);
  private:
    Tag(int ida, int bits, double sz, const PoseWithNoise &pe,
         const std::shared_ptr<Body> &body);

    // ------- variables --------------
    int            id_;            // tag id
    int            bits_{6};       // determines tag family
    double         size_;          // tag size in meters
    PoseWithNoise  poseWithNoise_; // tag pose relative body: T_b_o
    std::shared_ptr<Body> body_;   // body to which this tag belongs
    Eigen::Matrix<double, 4, 3> objectCorners_;
  };
  typedef Tag::TagPtr TagPtr;
  typedef Tag::TagConstPtr TagConstPtr;
  typedef Tag::TagVec TagVec;
  typedef std::map<int, TagPtr> TagMap;
  std::ostream &operator<<(std::ostream &os, const Tag &tag);
}
