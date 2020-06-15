/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <ros/ros.h>
#include "tagslam/approx_image_sync.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "approx_image_sync_node");
  ros::NodeHandle pnh("~");

  try {
    tagslam::ApproxImageSync node(pnh);
    node.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
    return (-1);
  }
  return (0);
}
