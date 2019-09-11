cmake_minimum_required(VERSION 2.8.3)
project(tagslam)

add_definitions(-std=c++11 -g -Wall -Wno-misleading-indentation -Wno-int-in-bool-context)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  nodelet
  rosbag
  flex_sync
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  apriltag_msgs
  apriltag_ros
  message_filters
  rosgraph_msgs
  cv_bridge
  tf
  tf_conversions
  eigen_conversions
  image_transport
)

find_package(Eigen3 REQUIRED QUIET)
find_package(OpenCV 3 REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS graph)

find_package(OpenMP)
IF(OPENMP_FOUND)
 SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_C_FLAGS}")
ENDIF()

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS geometry_msgs rosgraph_msgs roscpp flex_sync nodelet
	)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME}
#)

#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
#${OpenCV_LIBRARIES} ${GTSAM_LIBRARIES})

#add_dependencies(${PROJECT_NAME}
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
#)

#add_executable(${PROJECT_NAME}_node src/tag_slam_node.cpp)
#target_link_libraries(${PROJECT_NAME}_node
#  ${PROJECT_NAME} 
# ${catkin_LIBRARIES}
#)

add_executable(sync_and_detect_node src/sync_and_detect_node.cpp
src/sync_and_detect.cpp src/profiler.cpp)
target_link_libraries(sync_and_detect_node ${catkin_LIBRARIES})


add_library(tagslam
src/body.cpp
src/body_defaults.cpp
src/graph.cpp
src/graph_utils.cpp
src/gtsam_utils.cpp
src/geometry.cpp
src/pose_noise.cpp
src/tag_slam.cpp
src/yaml_utils.cpp
src/xml.cpp
src/simple_body.cpp src/board.cpp
src/tag.cpp
src/camera.cpp
src/pose_with_noise.cpp
src/gtsam_optimizer.cpp
src/value/pose.cpp
src/factor/absolute_pose_prior.cpp
src/factor/relative_pose_prior.cpp
src/factor/tag_projection.cpp
src/factor/distance.cpp
src/factor/coordinate.cpp
src/measurements/measurements.cpp
src/measurements/distance.cpp
src/measurements/coordinate.cpp
src/measurements/plane.cpp
src/graph_vertex.cpp
src/graph_edge.cpp
src/vertex.cpp
src/camera_intrinsics.cpp
src/odometry_processor.cpp
src/init_pose.cpp
src/rpp.cpp
src/quartic.cpp
src/profiler.cpp
src/graph_updater.cpp
src/gtsam_distortion/Cal3FS2.cpp
src/gtsam_distortion/Cal3DS3.cpp
)

target_link_libraries(tagslam ${catkin_LIBRARIES}  ${Boost_LIBRARIES}
${GTSAM_LIBRARIES})
#cotire(tagslam)

# -- nodelet ---
add_library(tagslam_nodelet src/tagslam_nodelet.cpp)
target_link_libraries(tagslam_nodelet tagslam ${catkin_LIBRARIES})
add_dependencies(tagslam_nodelet ${catkin_EXPORTED_TARGETS})
#cotire(tagslam_nodelet)

# -- node ---
add_executable(tagslam_node src/tagslam_node.cpp)
target_link_libraries(tagslam_node tagslam ${catkin_LIBRARIES})

# -- rpp test ---
add_executable(rpp_test src/rpp_test.cpp src/rpp.cpp src/quartic.cpp)
target_link_libraries(rpp_test ${catkin_LIBRARIES})

#cotire(tagslam_node)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
