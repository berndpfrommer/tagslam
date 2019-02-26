/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/odometry_processor.h"
#include "tagslam/geometry.h"
#include "tagslam/graph.h"

namespace tagslam {
  using Odometry = nav_msgs::Odometry;
  using OdometryConstPtr = nav_msgs::OdometryConstPtr;

  OdometryProcessor::OdometryProcessor(Graph *graph,
                                       const BodyConstPtr &body) :
    graph_(graph), body_(body) {
  }

  static Transform to_pose(const OdometryConstPtr &odom) {
    const geometry_msgs::Quaternion &q = odom->pose.pose.orientation;
    const geometry_msgs::Point      &p = odom->pose.pose.position;
    return (make_transform(Eigen::Quaterniond(q.w, q.x, q.y, q.z),
                           Point3d(p.x, p.y, p.z)));
  }

  void
  OdometryProcessor::process(const OdometryConstPtr &msg) {
    ROS_INFO_STREAM("odom processing msg " << msg->header.stamp);
    Transform newPose = to_pose(msg);
    if (time_ == ros::Time(0)) {
      pose_ = newPose;
      time_ = msg->header.stamp;
      ROS_INFO_STREAM("odom first time called, no body pose delta yet!");
      return;
    }
    Transform deltaPose = pose_.inverse() * newPose;
    PoseNoise2 dpn = PoseNoise2::make(0.01 /*angle*/, 0.01 /*position*/);
    const PoseWithNoise pn(deltaPose, dpn, true);
    ROS_INFO_STREAM("odom adding body pose: " << time_ << " -> " << msg->header.stamp);
    graph_->addBodyPoseDelta(time_, msg->header.stamp, body_, pn);
    
    pose_ = newPose;
    time_ = msg->header.stamp;
  }

} // end of namespace
