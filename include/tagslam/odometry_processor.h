/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/body.h"
#include "tagslam/geometry.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace tagslam {
  class Graph;
  class OdometryProcessor {
    using Odometry = nav_msgs::Odometry;
    using OdometryConstPtr = nav_msgs::OdometryConstPtr;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    OdometryProcessor(Graph *graph, const BodyConstPtr &body);
    void process(const OdometryConstPtr &msgs);

    Graph         *graph_{NULL};
    BodyConstPtr  body_;
    Transform     pose_;
    ros::Time     time_{0};
  };
}
