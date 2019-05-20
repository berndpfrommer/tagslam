/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <ros/ros.h>
#include <sstream>


namespace tagslam {

#define BOMB_OUT(...) \
  { ROS_ERROR_STREAM(__VA_ARGS__); \
    std::stringstream SS; \
    SS << __VA_ARGS__;                        \
    throw (std::runtime_error(SS.str())); }

}

