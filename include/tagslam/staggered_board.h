/* -*-c++-*--------------------------------------------------------------------
 * 2023 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/body.h"
#include <iostream>
#include <memory>

namespace tagslam {
  class StaggeredBoard: public Body {
  public:
    using string = std::string;
    typedef std::shared_ptr<StaggeredBoard> StaggeredBoardPtr;
    typedef std::shared_ptr<const StaggeredBoard> StaggeredBoardConstPtr;

    StaggeredBoard(const string &n  = string(""), bool iS = false) :
      Body(n, iS) {
      type_ = "board";
    }
    bool printTags() const override { return (false); }
    bool parse(XmlRpc::XmlRpcValue body, const BodyPtr &bp) override;
    bool write(std::ostream &os, const string &prefix) const override;
  private:
    int     tagStartId_{-1};
    double  tagSize_{-1.0};
    int     tagBits_{6};
    double  tagSpacing_{0.25};
    int     tagRows_{-1};
    int     tagColumns_{-1};
    double  tagRotationNoise_;
    double  tagPositionNoise_;
  };
  using StaggeredBoardPtr      = StaggeredBoard::StaggeredBoardPtr;
  using StaggeredBoardConstPtr = StaggeredBoard::StaggeredBoardConstPtr;
}
