/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_SYNC_AND_DETECT_H
#define TAGSLAM_SYNC_AND_DETECT_H

#include "tagslam/bag_sync.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
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
  class SyncAndDetect {
  public:
    SyncAndDetect(const ros::NodeHandle &pnh);
    ~SyncAndDetect();

    SyncAndDetect(const SyncAndDetect&) = delete;
    SyncAndDetect& operator=(const SyncAndDetect&) = delete;

    bool initialize();

  private:
    void processImages(const std::vector<ImageConstPtr> &msgvec);
    void processCompressedImages(const std::vector<CompressedImageConstPtr> &msgvec);
    void processCVMat(const std::vector<std_msgs::Header> &headers,
                      const std::vector<cv::Mat> &grey,
                      const std::vector<cv::Mat> &imgs);
    void processBag(const std::string &fname);
    template<typename T>
    void iterate_through_bag(
      const std::vector<std::string> &topics,
      rosbag::View *view,
      rosbag::Bag *bag,
      const std::function<void(const std::vector<boost::shared_ptr<T const>> &)> &callback)
      {
      BagSync<T> sync(topics, callback);
      for (const rosbag::MessageInstance &m: *view) {
        sync.process(m);
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
    bool                                imagesAreCompressed_{false};
    bool                                annotateImages_{false};
    int                                 maxFrameNumber_;
    apriltag_ros::ApriltagDetector::Ptr detector_;
  };
}

#endif
