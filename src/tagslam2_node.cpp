/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <ros/ros.h>
#include "tagslam/tag_slam2.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "tagslam2_node");
  ros::NodeHandle pnh("~");

  try {
    tagslam::TagSlam2 node(pnh);
    node.initialize();
    node.run();
    node.finalize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
