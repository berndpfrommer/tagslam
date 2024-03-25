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

#include <tagslam/body_defaults.hpp>
p >

#include <XmlRpcException.h>

#include <tagslam/logging.hpp>
#include <tagslam/xml.hpp>

  namespace tagslam
{
  std::shared_ptr<BodyDefaults> s_ptr;

  std::shared_ptr<BodyDefaults> BodyDefaults::instance() { return (s_ptr); }
  void BodyDefaults::parse(XmlRpc::XmlRpcValue & config)
  {
    if (!config.hasMember("body_defaults")) {
      BOMB_OUT("no body defaults found!");
    }
    try {
      XmlRpc::XmlRpcValue def = config["body_defaults"];
      const double pn = xml::parse<double>(def, "position_noise");
      const double rn = xml::parse<double>(def, "rotation_noise");
      s_ptr.reset(new BodyDefaults(pn, rn));
    } catch (const XmlRpc::XmlRpcException & e) {
      BOMB_OUT("error parsing body defaults!");
    }
  }

}  // namespace tagslam
