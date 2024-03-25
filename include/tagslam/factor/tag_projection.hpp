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

#ifndef TAGSLAM__RELATIVE_POSE_PRIOR_HPP_
#define TAGSLAM__RELATIVE_POSE_PRIOR_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <tagslam/factor/factor.hpp>
#include <tagslam/geometry.hpp>
#include <tagslam/pose_with_noise.hpp>

namespace tagslam
{
class Tag;
class Camera;
namespace factor
{
using std::string;
class TagProjection : public Factor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TagProjection(
    uint64_t t = 0,
    const std::shared_ptr<const Camera> & cam = std::shared_ptr<Camera>(),
    const std::shared_ptr<const Tag> & tag = std::shared_ptr<Tag>(),
    const geometry_msgs::msg::Point * imgCorn = NULL, double pixelNoise = 1.0,
    const string & name = "");
  // ------ inherited methods -----
  string getLabel() const override;
  VertexId getId() const override { return (make_id(time_, name_)); }
  std::shared_ptr<Vertex> clone() const override
  {
    return (std::shared_ptr<TagProjection>(new TagProjection(*this)));
  }
  VertexDesc addToGraph(const VertexPtr & vp, Graph * g) const override;
  void addToOptimizer(Graph * g) const override;
  bool establishesValues() const override { return (true); }
  // --------- own methods
  const Eigen::Matrix<double, 4, 2> & getImageCorners() const
  {
    return (imgCorners_);
  }
  const std::shared_ptr<const Camera> getCamera() const { return (cam_); }
  const std::shared_ptr<const Tag> getTag() const { return (tag_); }
  double getPixelNoise() const { return (pixelNoise_); }

private:
  const std::shared_ptr<const Camera> cam_;
  const std::shared_ptr<const Tag> tag_;
  double pixelNoise_;
  Eigen::Matrix<double, 4, 2> imgCorners_;
};
}  // namespace factor
typedef std::shared_ptr<factor::TagProjection> TagProjectionFactorPtr;
typedef std::shared_ptr<const factor::TagProjection>
  TagProjectionFactorConstPtr;
}  // namespace tagslam
#endif  // TAGSLAM__RELATIVE_POSE_PRIOR_HPP_