/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <ros/ros.h>
#include "tagslam/tag_slam.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "tagslam_node");
  ros::NodeHandle pnh("~");

  try {
    tagslam::TagSlam node(pnh);
    node.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
