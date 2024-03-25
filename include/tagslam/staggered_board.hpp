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

#ifndef TAGSLAM__STAGGERED_BOARD_HPP_
#define TAGSLAM__STAGGERED_BOARD_HPP_

#include <memory>
#include <tagslam/body.hpp>

namespace tagslam
{
class StaggeredBoard : public Body
{
public:
  using string = std::string;
  typedef std::shared_ptr<StaggeredBoard> StaggeredBoardPtr;
  typedef std::shared_ptr<const StaggeredBoard> StaggeredBoardConstPtr;

  StaggeredBoard(const string & n = string(""), bool iS = false) : Body(n, iS)
  {
    type_ = "staggered_board";
  }
  bool printTags() const override { return (false); }
  bool parse(const YAML::Node & body, const BodyPtr & bp) override;
  bool write(std::ostream & os, const string & prefix) const override;

private:
  int tagStartId_{-1};
  double tagSize_{-1.0};
  int tagBits_{6};
  double tagSpacing_{0.25};
  int tagRows_{-1};
  int tagColumns_{-1};
  double tagRotationNoise_;
  double tagPositionNoise_;
};
using StaggeredBoardPtr = StaggeredBoard::StaggeredBoardPtr;
using StaggeredBoardConstPtr = StaggeredBoard::StaggeredBoardConstPtr;
}  // namespace tagslam
#endif  // TAGSLAM__STAGGERED_BOARD_HPP_