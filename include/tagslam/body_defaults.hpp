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

#ifndef TAGSLAM__BODY_DEFAULTS_HPP_
#define TAGSLAM__BODY_DEFAULTS_HPP_

#include <memory>

namespace YAML
{
class Node;  // forward decl
}

namespace tagslam
{
struct BodyDefaults
{
  BodyDefaults(double p, double r) : positionNoise(p), rotationNoise(r) {}
  static std::shared_ptr<BodyDefaults> instance();
  static void parse(const YAML::Node & config);
  double positionNoise{1e-1};
  double rotationNoise{1e-1};
};
}  // namespace tagslam
#endif  // #define TAGSLAM__BODY_DEFAULTS_HPP_