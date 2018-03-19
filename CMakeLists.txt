cmake_minimum_required(VERSION 2.8.3)
project(tagslam)

add_definitions(-std=c++11 -g)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rosbag
  std_msgs
  geometry_msgs
  apriltag_msgs
  message_filters
  tf
)

find_package(Eigen3 REQUIRED QUIET)
find_package(OpenCV 3 REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)


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
src/tag_graph.cpp src/initial_pose_graph.cpp)

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

