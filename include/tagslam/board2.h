/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/body.h"
#include <iostream>

namespace tagslam {
  struct Board2: public Body {
    Board2(const std::string &n  = std::string(""), bool iS = false) :
      Body(n, iS) {
      type = "board";
    }
    bool parse(XmlRpc::XmlRpcValue body, const BodyPtr &bp) override;
    bool write(std::ostream &os, const std::string &prefix) const override;
    int     tagStartId{-1};
    double  tagSize{-1.0};
    int     tagBits{6};
    double  tagSpacing{0.25};
    int     tagRows{-1};
    int     tagColumns{-1};
    double  tagRotationNoise;
    double  tagPositionNoise;
    typedef std::shared_ptr<Board2> Board2Ptr;
    typedef std::shared_ptr<const Board2> Board2ConstPtr;
  };
  using Board2Ptr      = Board2::Board2Ptr;
  using Board2ConstPtr = Board2::Board2ConstPtr;
}
