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

  OdometryProcessor::OdometryProcessor(ros::NodeHandle &nh,
                                       Graph *graph,
                                       const BodyConstPtr &body) :
    graph_(graph), body_(body) {
    pub_ = nh.advertise<nav_msgs::Odometry>("raw_odom/body_"+body->getName(), 5);
    T_body_odom_ = body->getTransformBodyOdom();
  }

  static Transform to_pose(const OdometryConstPtr &odom) {
    const geometry_msgs::Quaternion &q = odom->pose.pose.orientation;
    const geometry_msgs::Point      &p = odom->pose.pose.position;
    return (make_transform(Eigen::Quaterniond(q.w, q.x, q.y, q.z),
                           Point3d(p.x, p.y, p.z)));
  }

  void
  OdometryProcessor::process(const OdometryConstPtr &msg) {
    auto msg2 = *msg;
    msg2.header.frame_id = "map";
    pub_.publish(msg2);
    ROS_INFO_STREAM("odom processing msg " << msg->header.stamp);
    Transform newPose = to_pose(msg);
    std::cout << "new pose: " << std::endl;
    std::cout << newPose << std::endl;
     
    if (time_ == ros::Time(0)) {
      pose_ = newPose;
      time_ = msg->header.stamp;
//#define INIT_POSE_WITH_IDENTITY
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
    //Transform deltaPose = newPose * pose_.inverse(); //pose_.inverse() * newPose;
    const auto &tf = T_body_odom_;
    Transform deltaPose = tf * pose_.inverse() * newPose * tf.inverse();
    std::cout << "delta pose:" << std::endl;
    std::cout << deltaPose << std::endl;
    PoseNoise2 dpn = PoseNoise2::make(0.01 /*angle*/, 0.01 /*position*/);
    const PoseWithNoise pn(deltaPose, dpn, true);
    graph_->addBodyPoseDelta(time_, msg->header.stamp, body_, pn);
    
    pose_ = newPose;
    time_ = msg->header.stamp;
  }

} // end of namespace
