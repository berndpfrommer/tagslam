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

#ifndef TAGSLAM__CAL3DS2U_HPP_
#define TAGSLAM__CAL3DS2U_HPP_

#include <gtsam/geometry/Cal3DS2.h>

class Cal3DS2U : public gtsam::Cal3DS2
{
public:
  Cal3DS2U(
    double fx, double fy, double s, double u0, double v0, double k1, double k2,
    double p1 = 0.0, double p2 = 0.0)
  : gtsam::Cal3DS2(fx, fy, s, u0, v0, k1, k2, p1, p2)
  {
  }
  explicit Cal3DS2U(const gtsam::Cal3DS2 & cal) : Cal3DS2(cal) {}

  gtsam::Point2 uncalibrate(
    const gtsam::Point2 & p, gtsam::OptionalJacobian<2, 9> Dcal = {},
    gtsam::OptionalJacobian<2, 2> Dp = {}) const
  {
    return (gtsam::Cal3DS2_Base::uncalibrate(p, Dcal, Dp));
  }
};
// now it gets even uglier, as I had to put this into the gtsam namespace
namespace gtsam
{
template <>
struct traits<Cal3DS2U> : public internal::Manifold<Cal3DS2>
{
};
template <>
struct traits<const Cal3DS2U> : public internal::Manifold<Cal3DS2>
{
};
}  // namespace gtsam

#endif  // TAGSLAM__CAL3DS2U_HPP_
