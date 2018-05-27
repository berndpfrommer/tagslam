/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_BOARD_H
#define TAGSLAM_BOARD_H

#include "tagslam/rigid_body.h"
#include <iostream>

namespace tagslam {
  struct Board: public RigidBody {
    Board(const std::string &n  = std::string(""),
          bool iS = false) : RigidBody(n, iS) {
      type = "board";
    }
    bool parse(XmlRpc::XmlRpcValue body_defaults,
               XmlRpc::XmlRpcValue body) override;
    bool write(std::ostream &os, const std::string &prefix) const override;
    int     tagStartId{-1};
    double  tagSize{-1.0};
    int     tagBits{6};
    double  tagSpacing{0.25};
    int     tagRows{-1};
    int     tagColumns{-1};
    double  tagRotationNoise;
    double  tagPositionNoise;
    typedef std::shared_ptr<Board> BoardPtr;
    typedef std::shared_ptr<const Board> BoardConstPtr;


  };
  using BoardPtr      = Board::BoardPtr;
  using BoardConstPtr = Board::BoardConstPtr;
}

#endif
