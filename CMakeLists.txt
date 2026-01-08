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

set(tagslam_packages
  "apriltag_msgs"
  "cv_bridge"
  "flex_sync"
  "geometry_msgs"
  "image_transport"
  "nav_msgs"
  "rclcpp"
  "rclcpp_components"
  "rosbag2_cpp"
  "rosbag2_transport"
  "rosgraph_msgs"
  "sensor_msgs"
  "std_srvs"
  "tf2_ros"
  "tf2_msgs"
  "tf2")

set(sync_and_detect_packages
  "apriltag_msgs"
  "apriltag_detector"
  "cv_bridge"
  "flex_sync"
  "image_transport"
  "nav_msgs"
  "pluginlib"
  "rclcpp"
  "sensor_msgs")

set(all_packages ${tagslam_packages} ${sync_and_detect_packages})

 foreach(pkg ${all_packages})
  find_package(${pkg} REQUIRED)
endforeach()

set(sync_and_detect_dependencies
 ${apriltag_msgs_TARGETS}
 apriltag_detector::apriltag_detector
 cv_bridge::cv_bridge
 flex_sync::flex_sync
 image_transport::image_transport
 ${nav_msgs_TARGETS}
 pluginlib::pluginlib
 rclcpp::rclcpp
 ${sensor_msgs_TARGETS})

set(tagslam_dependencies
 ${apriltag_msgs_TARGETS}
 cv_bridge::cv_bridge
 flex_sync::flex_sync
 ${geometry_msgs_TARGETS}
 gtsam
 image_transport::image_transport
 ${nav_msgs_TARGETS}
 opencv_core
 opencv_imgproc
 opencv_calib3d
 yaml-cpp
 rclcpp::rclcpp
 rclcpp_components::component
 rosbag2_cpp::rosbag2_cpp
 rosbag2_transport::rosbag2_transport
 ${rosgraph_msgs_TARGETS}
 ${sensor_msgs_TARGETS}
 ${std_srvs_TARGETS}
 tf2_ros::tf2_ros
 ${tf2_msgs_TARGETS}
 tf2::tf2)
 

 if(${image_transport_VERSION} VERSION_GREATER_EQUAL "6.3.0")
   add_definitions(-DIMAGE_TRANSPORT_USE_QOS)
endif()
if(${image_transport_VERSION} VERSION_GREATER_EQUAL "6.4.0")
  add_definitions(-DIMAGE_TRANSPORT_USE_NODEINTERFACE)
endif()


#
# --------- library
#
add_library(${PROJECT_NAME} SHARED
  src/board.cpp
  src/body_defaults.cpp
  src/body.cpp
  src/camera_intrinsics.cpp
  src/camera.cpp
  src/factor/absolute_pose_prior.cpp
  src/factor/coordinate.cpp
  src/factor/distance.cpp
  src/factor/relative_pose_prior.cpp
  src/factor/tag_projection.cpp
  src/geometry.cpp
  src/graph_edge.cpp
  src/graph_updater.cpp
  src/graph_utils.cpp
  src/graph_vertex.cpp
  src/graph.cpp
  src/gtsam_optimizer.cpp
  src/gtsam_utils.cpp
  src/gtsam_distortion/Cal3DS3.cpp
  src/gtsam_distortion/Cal3FS2.cpp
  src/init_pose.cpp
  src/measurements/coordinate.cpp
  src/measurements/distance.cpp
  src/measurements/measurements.cpp
  src/measurements/plane.cpp
  src/odometry_processor.cpp
  src/pose_noise.cpp
  src/pose_with_noise.cpp
  src/profiler.cpp
  src/quartic.cpp
  src/rpp.cpp
  src/simple_body.cpp
  src/staggered_board.cpp
  src/tag.cpp
  src/tagslam.cpp
  src/value/pose.cpp
  src/vertex.cpp
  src/yaml_utils.cpp
  src/yaml.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${tagslam_dependencies})
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
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_node PRIVATE include)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)

#
# -------- sync-and-detect shared library
#
add_library(sync_and_detect SHARED
  src/sync_and_detect.cpp
)
target_link_libraries(sync_and_detect yaml-cpp ${sync_and_detect_dependencies})
target_compile_features(sync_and_detect PRIVATE cxx_std_17)

target_include_directories(
    sync_and_detect
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

rclcpp_components_register_nodes(sync_and_detect "${PROJECT_NAME}::SyncAndDetect")

#
# -------- processing from bag
#
add_executable(${PROJECT_NAME}_from_bag
  src/${PROJECT_NAME}_from_bag.cpp
  src/enhanced_player.cpp)
target_link_libraries(${PROJECT_NAME}_from_bag ${PROJECT_NAME} sync_and_detect ${tagslam_dependencies})
target_include_directories(${PROJECT_NAME}_from_bag PRIVATE include)
target_compile_features(${PROJECT_NAME}_from_bag PRIVATE cxx_std_17)

#
# -------- sync-and-detect from bag
#

add_executable(sync_and_detect_from_bag
  src/sync_and_detect_from_bag.cpp
  src/enhanced_player.cpp)
target_link_libraries(sync_and_detect_from_bag sync_and_detect yaml-cpp rosbag2_transport::rosbag2_transport)

target_include_directories(sync_and_detect_from_bag PRIVATE include)
target_compile_features(sync_and_detect_from_bag PRIVATE cxx_std_17)

#
# -------- sync-and-detect node
#

add_executable(sync_and_detect_node src/sync_and_detect_node.cpp)
target_link_libraries(sync_and_detect_node sync_and_detect)

target_include_directories(sync_and_detect_node PRIVATE include)
target_compile_features(sync_and_detect_node PRIVATE cxx_std_17)

#
# --------------------- installation -----------------------------
#
# the node must go into the project specific lib directory or else
# the launch file will not find it

install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_from_bag
  sync_and_detect_node
  sync_and_detect_from_bag
  DESTINATION lib/${PROJECT_NAME}/)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  ${PROJECT_NAME}
  sync_and_detect
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
  # find_package(ament_cmake_flake8 REQUIRED)
  # find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  # ament_flake8()
  # ament_pep257()
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
