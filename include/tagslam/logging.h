/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <ros/ros.h>
#include <sstream>


namespace tagslam {

#define BOMB_OUT(X) \
  { ROS_ERROR_STREAM(X); std::stringstream SS; SS << X; \
    throw (std::runtime_error(SS.str())); }

}

