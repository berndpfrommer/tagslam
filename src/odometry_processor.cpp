/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/odometry_processor.h"
#include "tagslam/geometry.h"
#include "tagslam/graph.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

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
#define INIT_POSE_WITH_IDENTITY
#ifdef INIT_POSE_WITH_IDENTITY
      std::string name = Graph::body_name(body_->getName());
      Graph::VertexPose vp = graph_->findPose(time_, name);
      PoseWithNoise pn(Transform::Identity(),
                       PoseNoise2::make(0.1, 0.1), true);
      if (!vp.pose) {
        graph_->addPoseWithPrior(time_, name, pn);
      } else {
        graph_->addPrior(time_, vp, name,pn);
      }
#endif      
    }
    Transform deltaPose = pose_.inverse() * newPose;
    PoseNoise2 dpn = PoseNoise2::make(0.01 /*angle*/, 0.01 /*position*/);
    const PoseWithNoise pn(deltaPose, dpn, true);
    graph_->addBodyPoseDelta(time_, msg->header.stamp, body_, pn);
    
    pose_ = newPose;
    time_ = msg->header.stamp;
  }

} // end of namespace
