/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/odometry_processor.h"
#include "tagslam/geometry.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <fstream>

namespace tagslam {
  using Odometry = nav_msgs::Odometry;
  using OdometryConstPtr = nav_msgs::OdometryConstPtr;
  //std::ofstream debug_file("odom_debug.txt");

  OdometryProcessor::OdometryProcessor(ros::NodeHandle &nh,
                                       const BodyConstPtr &body) :
    body_(body) {
    pub_ =
      nh.advertise<nav_msgs::Odometry>("raw_odom/body_"+body->getName(), 5);
    accelerationNoiseMin_ = body->getOdomAccelerationNoiseMin();
    angularAccelerationNoiseMin_ = body->getOdomAngularAccelerationNoiseMin();
    accelerationNoiseMax_ = body->getOdomAccelerationNoiseMax();
    angularAccelerationNoiseMax_ = body->getOdomAngularAccelerationNoiseMax();
    rotationNoise_ = body->getOdomRotationNoise();
    translationNoise_ = body->getOdomTranslationNoise();
    T_body_odom_ = body->getTransformBodyOdom();
  }

  static Transform to_pose(const OdometryConstPtr &odom) {
    const geometry_msgs::Quaternion &q = odom->pose.pose.orientation;
    const geometry_msgs::Point      &p = odom->pose.pose.position;
    return (make_transform(Eigen::Quaterniond(q.w, q.x, q.y, q.z),
                           Point3d(p.x, p.y, p.z)));
  }

  PoseNoiseConstPtr OdometryProcessor::makeAdaptiveNoise(
    const ros::Time &t, const Transform &deltaPose)  {
   
    const double dt = std::max((t - time_).toSec(), 0.001);
    const double dt2 = dt * dt;
    const double dtinv = 1.0 / dt;
    const Eigen::Vector3d dx = deltaPose.translation();
    Eigen::AngleAxisd aa;
    aa.fromRotationMatrix(deltaPose.rotation());
    // da is the incremental rotation vector
    const Eigen::Vector3d da = aa.angle() * aa.axis();
    // compute instantaneous velocity of the last update
    const Eigen::Vector3d v = dx * dtinv;
    // compute instantaneous angular velocity of the last update
    const Eigen::Vector3d w = da * dtinv;
    // compute error in position, i.e. how much the position change
    // deviates from what was expected assuming constant velocity
    const double dpos = (dx - lastVelocity_ * dt).norm();
    // same now for angle error
    const double dang = (da - lastOmega_ * dt).norm();
    lastOmega_    = w;
    lastVelocity_ = v;
    // Whenever the accelerations go above the
    // typical values, the noise will increase correspondingly,
    // thereby reducing the weight of the odometry measurement.
    // This addresses situations where the odometry jumps.
    //
    // There are parameters that clamp the noise, i.e. they set a floor
    // and ceiling on the position and angle uncertainty. Without a floor,
    // the noise could go to zero if the position update is zero (no motion),
    // meaning the odometry measurements are trusted completely, causing
    // the optimizer to bomb out.
    
    const double angNoise =
      std::min(std::max(dang,angularAccelerationNoiseMin_ * dt2),
               angularAccelerationNoiseMax_ * dt2);
    const double posNoise =
      std::min(std::max(dpos, accelerationNoiseMin_ * dt2),
               accelerationNoiseMax_ * dt2);
    
    //debug_file <<  t  << " " << posNoise << " " << accelerationNoiseMin_ * dt2
    //<< " " << accelerationNoiseMax_ * dt2 << std::endl;
    PoseNoiseConstPtr pn(new PoseNoise(PoseNoise::make(angNoise, posNoise)));
    return (pn);
  }


  void
  OdometryProcessor::updateStatistics(const ros::Time &t, const Transform &d) {
    const double l2 = d.translation().squaredNorm();
    Eigen::AngleAxisd aa; // angle-axis
    aa.fromRotationMatrix(d.rotation());
    const double a = aa.angle();
    const double l = std::sqrt(l2);
    lenSum_  += l;
    len2Sum_ += l2;
    if (l >= lenMax_) {
      lenMax_ = l;
      lenMaxT_ = t;
    }
    angSum_  += a;
    ang2Sum_ += a * a;
    count_++;
    ROS_DEBUG_STREAM("odom: " << a << " l: " << l);
  }

  void
  OdometryProcessor::finalize() const {
    const double cinv = (count_ > 0) ? (1.0/(double)count_) : 0.0;
    const double lavg = lenSum_ * cinv;
    const double cov  = cinv * (len2Sum_ - lavg * lavg * (double)count_);
    const double aavg = angSum_ * cinv;
    const double acov = cinv * (ang2Sum_ - aavg * aavg * (double)count_);
    ROS_INFO_STREAM("---- odom statistics: ");
    ROS_INFO_STREAM("translation: " << lavg << " +- " << std::sqrt(cov));
    ROS_INFO_STREAM("max: " << lenMax_ << " at time: " << lenMaxT_);
    ROS_INFO_STREAM("rotation: " << aavg << " +- " << std::sqrt(acov));
  }

  void
  OdometryProcessor::process(Graph *graph, const OdometryConstPtr &msg,
                             std::vector<VertexDesc> *factors) {
    auto msg2 = *msg;
    msg2.header.frame_id = "map";
    pub_.publish(msg2);
    Transform newPose = to_pose(msg);
    const ros::Time &t = msg->header.stamp;
    if (time_ == ros::Time(0)) {
      lastOmega_ = Eigen::Vector3d(0, 0, 0);
      lastVelocity_ = Eigen::Vector3d(0, 0, 0);
    } else {
      const auto  &tf = T_body_odom_;
      Transform deltaPose = tf * pose_.inverse() * newPose * tf.inverse();
      PoseNoiseConstPtr pn;
      if (rotationNoise_ <= 0.0 || translationNoise_ <= 0.0) {
        pn = makeAdaptiveNoise(t, deltaPose);
      } else {
        pn.reset(
          new PoseNoise(PoseNoise::make(rotationNoise_, translationNoise_)));
      }
      const PoseWithNoise pwn(deltaPose, *pn, true);
      updateStatistics(t, deltaPose);
      auto fac = add_body_pose_delta(graph, time_, msg->header.stamp,
                                     body_, pwn);
      factors->push_back(fac);
    }
    pose_ = newPose;
    time_ = t;
  }

  VertexDesc
  OdometryProcessor::add_body_pose_delta(
    Graph *graph, const ros::Time &tPrev,
    const ros::Time &tCurr, const BodyConstPtr &body,
    const PoseWithNoise &deltaPose) {
    Transform prevPose;
    const std::string name = Graph::body_name(body->getName());
    const VertexDesc pp = graph->findPose(tPrev, name);
    const VertexDesc cp = graph->findPose(tCurr, name);
    if (!Graph::is_valid(pp)) {
      ROS_DEBUG_STREAM("adding previous pose for " << name << " " << tPrev);
      graph->addPose(tPrev, name, false);
    }
    if (!Graph::is_valid(cp)) {
      ROS_DEBUG_STREAM("adding current pose for " << name << " " << tCurr);
      graph->addPose(tCurr, name, false);
    }
    RelativePosePriorFactorPtr
      fac(new factor::RelativePosePrior(tCurr, tPrev, deltaPose, name));
    return (fac->addToGraph(fac, graph));
  }



} // end of namespace
