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

#pragma once

#include <tagslam/factor/coordinate.hpp>
#include <tagslam/measurements/measurements.hpp>
#include <tagslam/vertex.hpp>
#include <vector>

namespace tagslam
{
namespace measurements
{
class Coordinate : public Measurements
{
public:
  typedef std::shared_ptr<Coordinate> CoordinateMeasurementsPtr;

  void writeDiagnostics(const GraphPtr & graph) override;

  // static functions
  static CoordinateMeasurementsPtr read(
    XmlRpc::XmlRpcValue config, TagFactory * fac);

private:
};
}  // namespace measurements
}  // namespace tagslam
