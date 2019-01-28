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
  class Tag2 {
  public:
    //
    typedef std::shared_ptr<Tag2>        Tag2Ptr;
    typedef std::shared_ptr<const Tag2>  Tag2ConstPtr;
    typedef std::vector<Tag2Ptr>         Tag2Vec;

    int    getId()   const { return (id); }
    int    getBits() const { return (bits); }
    double getSize() const { return (size); }
    
    const PoseWithNoise &getPoseWithNoise() const { return (poseWithNoise); }

    Point3d getObjectCorner(int i) const;

    friend std::ostream &operator<<(std::ostream &os, const Tag2 &tag);
    // ----------- static methods
    static Tag2Vec parseTags(XmlRpc::XmlRpcValue xmltags, double sz);
    static Tag2Ptr  make(int ida, int bits, double sz,
                         const PoseWithNoise &pe = PoseWithNoise());
  private:
    Tag2(int ida, int bits, double sz, const PoseWithNoise &pe);

    // ------- variables --------------
    int            id;            // tag id
    int            bits{6};       // determines tag family
    double         size;          // tag size in meters
    PoseWithNoise  poseWithNoise; // tag pose relative body: T_b_o
    std::vector<Point3d>  objectCorners;  // 3d object coordinates
  };
  typedef Tag2::Tag2Ptr Tag2Ptr;
  typedef Tag2::Tag2ConstPtr Tag2ConstPtr;
  typedef Tag2::Tag2Vec Tag2Vec;
  typedef std::map<int, Tag2Ptr> Tag2Map;
  std::ostream &operator<<(std::ostream &os, const Tag2 &tag);
}
