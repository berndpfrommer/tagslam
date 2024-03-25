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

#ifndef TAGSLAM__SIMPLE_BODY_HPP_
#define TAGSLAM__SIMPLE_BODY_HPP_

#include <iostream>
#include <tagslam/body.hpp>

namespace YAML
{
class Node;  // forward decl
}

namespace tagslam
{
struct SimpleBody : public Body
{
  SimpleBody(const std::string & n = std::string(""), bool iS = false)
  : Body(n, iS)
  {
    type_ = "simple";
  }
  typedef std::shared_ptr<SimpleBody> SimpleBodyPtr;
  typedef std::shared_ptr<const SimpleBody> SimpleBodyConstPtr;

  bool parse(const YAML::Node & body, const BodyPtr & bp) override;
  bool write(std::ostream & os, const std::string & prefix) const override;
};
using SimpleBodyPtr = SimpleBody::SimpleBodyPtr;
using SimpleBodyConstPtr = SimpleBody::SimpleBodyConstPtr;
}  // namespace tagslam

#endif  // SIMPLE_BODY_HPP_