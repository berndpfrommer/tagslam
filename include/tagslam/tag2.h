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
  class Tag2 {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //
    typedef std::shared_ptr<Tag2>        Tag2Ptr;
    typedef std::shared_ptr<const Tag2>  Tag2ConstPtr;
    typedef std::vector<Tag2Ptr>         Tag2Vec;

    int    getId()   const { return (id_); }
    int    getBits() const { return (bits_); }
    double getSize() const { return (size_); }
    const std::shared_ptr<Body> getBody() const { return (body_); }
    const PoseWithNoise &getPoseWithNoise() const { return (poseWithNoise_); }
    const Eigen::Matrix<double, 4, 3> &getObjectCorners()
      const { return (objectCorners_); }
    const Point3d getObjectCorner(int idx)
      const { return (idx >= 0 ? objectCorners_.row(idx) : Point3d(0,0,0)); }

    friend std::ostream &operator<<(std::ostream &os, const Tag2 &tag);
    // ----------- static methods
    static Tag2Vec parseTags(XmlRpc::XmlRpcValue xmltags, double sz,
                             const std::shared_ptr<Body> &body);
    static Tag2Ptr make(int ida, int bits, double sz, const PoseWithNoise &pe,
                        const std::shared_ptr<Body> &body);
  private:
    Tag2(int ida, int bits, double sz, const PoseWithNoise &pe,
         const std::shared_ptr<Body> &body);

    // ------- variables --------------
    int            id_;            // tag id
    int            bits_{6};       // determines tag family
    double         size_;          // tag size in meters
    PoseWithNoise  poseWithNoise_; // tag pose relative body: T_b_o
    std::shared_ptr<Body> body_;   // body to which this tag belongs
    Eigen::Matrix<double, 4, 3> objectCorners_;
  };
  typedef Tag2::Tag2Ptr Tag2Ptr;
  typedef Tag2::Tag2ConstPtr Tag2ConstPtr;
  typedef Tag2::Tag2Vec Tag2Vec;
  typedef std::map<int, Tag2Ptr> Tag2Map;
  std::ostream &operator<<(std::ostream &os, const Tag2 &tag);
}
