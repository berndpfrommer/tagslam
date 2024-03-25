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

#ifndef TAGSLAM__LOGGING_HPP_
#define TAGSLAM__LOGGING_HPP_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#define BOMB_OUT(...)                               \
  {                                                 \
    RCLCPP_ERROR_STREAM(get_logger(), __VA_ARGS__); \
    std::stringstream SS;                           \
    SS << __VA_ARGS__;                              \
    throw(std::runtime_error(SS.str()));            \
  }
#define LOG_INFO(...)                              \
  {                                                \
    RCLCPP_INFO_STREAM(get_logger(), __VA_ARGS__); \
  }
#define LOG_INFO_FMT(...)                   \
  {                                         \
    RCLCPP_INFO(get_logger(), __VA_ARGS__); \
  }
#define LOG_WARN(...)                              \
  {                                                \
    RCLCPP_WARN_STREAM(get_logger(), __VA_ARGS__); \
  }
#define LOG_ERROR(...)                              \
  {                                                 \
    RCLCPP_ERROR_STREAM(get_logger(), __VA_ARGS__); \
  }
#define LOG_DEBUG(...)                              \
  {                                                 \
    RCLCPP_DEBUG_STREAM(get_logger(), __VA_ARGS__); \
  }

#endif  // TAGSLAM__LOGGING_HPP_
