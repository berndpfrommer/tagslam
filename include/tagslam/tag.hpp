// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TAGSLAM__TAG_HPP_
#define TAGSLAM__TAG_HPP_

#include <iostream>
#include <map>
#include <memory>
#include <tagslam/geometry.hpp>
#include <tagslam/pose_with_noise.hpp>
#include <vector>

namespace YAML
{
class Node;  // forward decl
}

namespace tagslam
{
class Body;
class Tag
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //
  typedef std::shared_ptr<Tag> TagPtr;
  typedef std::shared_ptr<const Tag> TagConstPtr;
  typedef std::vector<TagPtr> TagVec;

  int getId() const { return (id_); }
  int getBits() const { return (bits_); }
  double getSize() const { return (size_); }
  const std::shared_ptr<Body> getBody() const { return (body_); }
  const PoseWithNoise & getPoseWithNoise() const { return (poseWithNoise_); }
  const Eigen::Matrix<double, 4, 3> & getObjectCorners() const
  {
    return (objectCorners_);
  }
  const Point3d getObjectCorner(int idx) const
  {
    return (idx >= 0 ? objectCorners_.row(idx) : Point3d(0, 0, 0));
  }

  friend std::ostream & operator<<(std::ostream & os, const Tag & tag);
  // ----------- static methods
  static TagVec parseTags(
    const YAML::Node & conf, double sz, const std::shared_ptr<Body> & body);
  static TagPtr make(
    int ida, int bits, double sz, const PoseWithNoise & pe,
    const std::shared_ptr<Body> & body);

private:
  Tag(
    int ida, int bits, double sz, const PoseWithNoise & pe,
    const std::shared_ptr<Body> & body);

  // ------- variables --------------
  int id_;                       // tag id
  int bits_{6};                  // determines tag family
  double size_;                  // tag size in meters
  PoseWithNoise poseWithNoise_;  // tag pose relative body: T_b_o
  std::shared_ptr<Body> body_;   // body to which this tag belongs
  Eigen::Matrix<double, 4, 3> objectCorners_;
};
typedef Tag::TagPtr TagPtr;
typedef Tag::TagConstPtr TagConstPtr;
typedef Tag::TagVec TagVec;
typedef std::map<int, TagPtr> TagMap;
std::ostream & operator<<(std::ostream & os, const Tag & tag);
}  // namespace tagslam

#endif  // TAGSLAM__TAG_HPP_