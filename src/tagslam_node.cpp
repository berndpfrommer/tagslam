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
    if (!node.initialize()) {
      ROS_ERROR_STREAM("init failed!");
      ros::shutdown();
      return (-1);
    }
    if (node.runOnline()) {
      node.subscribe();
      ros::spin();
    } else {
      try {
        node.run();
        node.finalize();
      } catch (const tagslam::OptimizerException &e) {
        ROS_ERROR_STREAM("exited with error!");
        return (-1);
      }
      bool exitWhenDone;
      pnh.param<bool>("exit_when_done", exitWhenDone, false);
      if (!exitWhenDone) {
        ros::spin(); // wait for potential replay service call
      }
    }
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
    return (-1);
  }
  return (0);
}
