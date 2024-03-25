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

#ifndef TAGSLAM__VERTEX_HPP_
#define TAGSLAM__VERTEX_HPP_

#include <iostream>
#include <memory>
#include <string>
#include <tagslam/boost_graph.hpp>
#include <tagslam/optimizer_key.hpp>
#include <tagslam/vertex_desc.hpp>

namespace tagslam
{
// need forward declaration here
typedef std::string VertexId;
class Graph;
class Vertex
{
public:
  typedef std::shared_ptr<Vertex> VertexPtr;
  typedef std::shared_ptr<const Vertex> VertexConstPtr;
  Vertex(const std::string & n = "", const std::string & s = "", uint64_t t = 0)
  : name_(n), shape_(s), time_(t)
  {
  }
  virtual ~Vertex(){};
  virtual bool isValue() const = 0;
  virtual std::shared_ptr<Vertex> clone() const = 0;
  virtual VertexId getId() const = 0;
  virtual VertexDesc addToGraph(const VertexPtr & vp, Graph * g) const = 0;

  virtual std::string getLabel() const { return (name_); }
  virtual std::string getShape() const { return (shape_); }
  virtual std::string toString() const;

  const std::string & getName() const { return (name_); }

  uint64_t getTime() const { return (time_); }
  friend std::ostream & operator<<(std::ostream & os, const Vertex & v);
  inline static VertexId make_id(uint64_t t, const std::string & name)
  {
    return (name + "_" + std::to_string(t));
  }

protected:
  void checkIfValid(const VertexDesc & v, const std::string & m) const;
  // ----
  static std::string format_time(uint64_t t);
  std::string name_;
  std::string shape_;
  uint64_t time_;
};
typedef Vertex::VertexPtr VertexPtr;
typedef Vertex::VertexConstPtr VertexConstPtr;
std::ostream & operator<<(std::ostream & os, const Vertex & v);
}  // namespace tagslam

#endif  // TAGSLAM__VERTEX_HPP_