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

#ifndef TAGSLAM__FACTOR_HPP_
#define TAGSLAM__FACTOR_HPP_

#include <string>
#include <tagslam/factor_key.hpp>
#include <tagslam/vertex.hpp>

namespace tagslam
{
namespace factor
{
using std::string;
class Factor : public Vertex
{
public:
  Factor(const string & s = "", uint64_t t = 0) : Vertex(s, "box", t) {}
  virtual ~Factor() {}
  virtual void addToOptimizer(Graph * g) const = 0;
  virtual bool establishesValues() const = 0;
  bool isValue() const override { return (false); }

protected:
};
template <typename T>
static bool is(const VertexConstPtr & vp)
{
  return (std::dynamic_pointer_cast<const T>(vp) != 0);
}
inline static std::shared_ptr<const factor::Factor> cast_const(
  const VertexPtr & vp)
{
  return (std::dynamic_pointer_cast<const factor::Factor>(vp));
}
inline static std::shared_ptr<factor::Factor> cast(const VertexPtr & vp)
{
  return (std::dynamic_pointer_cast<factor::Factor>(vp));
}
}  // namespace factor
typedef std::shared_ptr<factor::Factor> FactorPtr;
typedef std::shared_ptr<const factor::Factor> FactorConstPtr;
}  // namespace tagslam

#endif  // TAGSLAM__FACTOR_HPP_