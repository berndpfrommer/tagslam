/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/body.h"
#include "tagslam/geometry.h"
#include "tagslam/pose_noise.h"
#include "tagslam/graph.h"


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace tagslam {
  class OdometryProcessor {
    using Odometry = nav_msgs::Odometry;
    using OdometryConstPtr = nav_msgs::OdometryConstPtr;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    OdometryProcessor(ros::NodeHandle &nh, const BodyConstPtr &body);
    void process(Graph *g, const OdometryConstPtr &msgs,
                 std::vector<VertexDesc> *factors);
    static VertexDesc add_body_pose_delta(
      Graph *graph, const ros::Time &tPrev, const ros::Time &tCurr,
      const BodyConstPtr &body, const PoseWithNoise &deltaPose);
    void finalize() const;
  private:
    PoseNoiseConstPtr makeAdaptiveNoise(const ros::Time &t,
                                        const Transform &deltaPose);
    void updateStatistics(const ros::Time &t, const Transform &d);
    // ---- variables
    BodyConstPtr    body_;
    Transform       pose_;
    ros::Time       time_{0};
    ros::Publisher  pub_;
    Transform       T_body_odom_;
    double          accelerationNoiseMin_{5.0}; // m/s^2
    double          angularAccelerationNoiseMin_{5.0}; // rad/sec^2
    double          accelerationNoiseMax_{50.0}; // m/s^2
    double          angularAccelerationNoiseMax_{50.0}; // rad/sec^2
    double          translationNoise_{-1.0}; // m
    double          rotationNoise_{-1.0}; // rads
    Eigen::Vector3d lastOmega_;
    Eigen::Vector3d lastVelocity_;
    double          lenSum_{0};
    double          len2Sum_{0};
    double          angSum_{0};
    double          ang2Sum_{0};
    unsigned int    count_{0};
    double          lenMax_{0};
    ros::Time       lenMaxT_{0};
  };
}
