/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/body.h"
#include <iostream>
#include <memory>

namespace tagslam {
  class Board2: public Body {
  public:
    typedef std::shared_ptr<Board2> Board2Ptr;
    typedef std::shared_ptr<const Board2> Board2ConstPtr;

    Board2(const std::string &n  = std::string(""), bool iS = false) :
      Body(n, iS) {
      type = "board";
    }
    bool parse(XmlRpc::XmlRpcValue body, const BodyPtr &bp) override;
    bool write(std::ostream &os, const std::string &prefix) const override;
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
  using Board2Ptr      = Board2::Board2Ptr;
  using Board2ConstPtr = Board2::Board2ConstPtr;
}
