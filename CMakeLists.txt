cmake_minimum_required(VERSION 2.8.3)
project(tagslam)

add_definitions(-std=c++11 -g -Wall)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rosbag
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  apriltag_msgs
  apriltag_ros
  message_filters
  cv_bridge
  tf
  tf_conversions
  eigen_conversions
)

find_package(Eigen3 REQUIRED QUIET)
find_package(OpenCV 3 REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

find_package(OpenMP)
IF(OPENMP_FOUND)
 SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_C_FLAGS}")
ENDIF()



catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS geometry_msgs roscpp
	)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/tag_slam.cpp src/camera.cpp
src/tag.cpp src/yaml_utils.cpp src/utils.cpp src/tag.cpp
src/pose_change.cpp
src/pose_noise.cpp
src/rigid_body.cpp
src/simple_body.cpp
src/board.cpp
src/pose_estimate.cpp
src/distance_measurement.cpp
src/position_measurement.cpp
src/profiler.cpp
src/tag_graph.cpp src/initial_pose_graph.cpp
src/gtsam_equidistant/Cal3FS2.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GTSAM_LIBRARIES})

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/tag_slam_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME} 
 ${catkin_LIBRARIES}
)

add_executable(sync_and_detect_node src/sync_and_detect_node.cpp
src/sync_and_detect.cpp)
target_link_libraries(sync_and_detect_node ${catkin_LIBRARIES})
