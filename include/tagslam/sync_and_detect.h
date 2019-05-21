/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <flex_sync/sync.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h>
#include <apriltag_ros/apriltag_detector.h>
#include <rosbag/bag.h>
#include <vector>
#include <memory>
#include <string>

namespace tagslam {
  using Image = sensor_msgs::Image;
  using ImageConstPtr = sensor_msgs::ImageConstPtr;
  using CompressedImage = sensor_msgs::CompressedImage;
  using CompressedImageConstPtr = sensor_msgs::CompressedImageConstPtr;
  using Odometry = nav_msgs::Odometry;
  using OdometryConstPtr = nav_msgs::OdometryConstPtr;
  class SyncAndDetect {
  public:
    SyncAndDetect(const ros::NodeHandle &pnh);
    ~SyncAndDetect();

    SyncAndDetect(const SyncAndDetect&) = delete;
    SyncAndDetect& operator=(const SyncAndDetect&) = delete;

    bool initialize();

  private:
    void processImages(const std::vector<ImageConstPtr> &msgvec,
                       const std::vector<OdometryConstPtr> &odomvec);
    void processCompressedImages(const std::vector<CompressedImageConstPtr> &mv,
                                 const std::vector<OdometryConstPtr> &odomvec);
    void processCVMat(const std::vector<std_msgs::Header> &headers,
                      const std::vector<cv::Mat> &grey,
                      const std::vector<cv::Mat> &imgs);
    void processBag(const std::string &fname);
    template<typename T>
    void iterate_through_bag(
      const std::vector<std::string> &topics,
      const std::vector<std::string> &odomTopics,
      rosbag::View *view,
      rosbag::Bag *bag,
      const std::function<void(const std::vector<boost::shared_ptr<T const>> &,
                               const std::vector<OdometryConstPtr> &)> &cb)  {

      std::vector<std::vector<std::string>> tpv;
      tpv.push_back(topics);
      tpv.push_back(odomTopics);
      flex_sync::Sync<T, Odometry> sync(tpv, cb);
      for (const rosbag::MessageInstance &m: *view) {
        OdometryConstPtr odom = m.instantiate<Odometry>();
        if (odom) {
          sync.process(m.getTopic(), odom);
        } else {
          boost::shared_ptr<T const> img = m.instantiate<T>();
          if (img) {
            sync.process(m.getTopic(), img);
          }
        }
        if (!ros::ok()) {
          break;
        }
        if ((int)fnum_ > maxFrameNumber_) {
          break;
        }
      }
    }

    // ----------------------------------------------------------
    ros::NodeHandle                     nh_;
    unsigned int                        fnum_{0};
    rosbag::Bag                         outbag_;
    std::vector<std::string>            tagTopics_;
    std::vector<std::string>            imageTopics_;
    std::vector<std::string>            odometryTopics_;
    std::vector<std::string>            imageOutputTopics_;
    bool                                imagesAreCompressed_{false};
    bool                                annotateImages_{false};
    int                                 maxFrameNumber_;
    int                                 skip_{1};
    apriltag_ros::ApriltagDetector::Ptr detector_;
    std::string                         detectorType_;
  };
}
