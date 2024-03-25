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

#ifndef TAGSLAM__TAGSLAM__VALUE_HPP_
#define TAGSLAM__TAGSLAM__VALUE_HPP_

#include <tagslam/value_key.hpp>
#include <tagslam/vertex.hpp>

namespace tagslam
{
namespace value
{
class Value : public Vertex
{
public:
  Value(const std::string & s = "", uint64_t t = 0) : Vertex(s, "ellipse", t) {}
  virtual ~Value() {}
  bool isValue() const override { return (true); }
  virtual std::string getLabel() const override { return (Vertex::getLabel()); }
};
}  // namespace value
typedef std::shared_ptr<value::Value> ValuePtr;
typedef std::shared_ptr<const value::Value> ValueConstPtr;

}  // namespace tagslam
#endif  // TAGSLAM__VALUE_HPP_