#
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(tagslam)

set(USE_ADDRESS_SANITIZER OFF CACHE BOOL "compile with address sanitizer support")
if(USE_ADDRESS_SANITIZER)
  set(CMAKE_CXX_FLAGS "-fsanitize=address  -fsanitize=leak -g")
  set(CMAKE_C_FLAGS "-fsanitize=leak -g")
  set(CMAKE_EXE_LINKER_FLAGS "-fsanitize=address  -fsanitize=leak")
  set(CMAKE_MODULE_LINKER_FLAGS "-fsanitize=address  -fsanitize=leak")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(Eigen3 REQUIRED QUIET)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS graph)
find_package(GTSAM REQUIRED)

set(ament_dependencies
  "apriltag_msgs"
  "cv_bridge"
  "flex_sync"
  "geometry_msgs"
  "image_transport"
  "nav_msgs"
  "rclcpp"
  "rclcpp_components"
  "rosbag2_cpp"
  "sensor_msgs"
  "tf2_ros"
  "std_srvs")

foreach(pkg ${ament_dependencies})
  find_package(${pkg} REQUIRED)
endforeach()

# find_package(OpenMP)
# IF(OPENMP_FOUND)
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_C_FLAGS}")
# ENDIF()


#
# --------- library
#
add_library(${PROJECT_NAME} SHARED
  src/tagslam.cpp
  src/tag.cpp
  src/body.cpp
  src/pose_with_noise.cpp
  src/pose_noise.cpp
  src/board.cpp
  src/graph_utils.cpp
  src/graph.cpp
  src/vertex.cpp
  src/value/pose.cpp
  src/factor/absolute_pose_prior.cpp
  src/gtsam_optimizer.cpp
  src/gtsam_utils.cpp
  src/gtsam_distortion/Cal3DS3.cpp
  src/gtsam_distortion/Cal3FS2.cpp
  src/yaml_utils.cpp
  src/simple_body.cpp
  src/staggered_board.cpp
  src/yaml.cpp
  src/geometry.cpp
  src/factor/tag_projection.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${ament_dependencies})
target_link_libraries(${PROJECT_NAME} opencv_core opencv_imgproc opencv_calib3d yaml-cpp gtsam)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_include_directories(
    ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::TagSLAM")

#
# -------- node
#
add_executable(${PROJECT_NAME}_node src/tagslam_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${ament_dependencies})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_node PRIVATE include)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)

# the node must go into the project specific lib directory or else
# the launch file will not find it

install(TARGETS
  ${PROJECT_NAME}_node

  DESTINATION lib/${PROJECT_NAME}/)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}/)
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py")

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_xmllint()

endif()

ament_package()
