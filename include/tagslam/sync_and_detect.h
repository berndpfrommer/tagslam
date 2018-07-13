/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_SYNC_AND_DETECT_H
#define TAGSLAM_SYNC_AND_DETECT_H

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

    void processImages(const std::vector<ImageConstPtr> &msgvec);
    void processCompressedImages(const std::vector<CompressedImageConstPtr> &msgvec);
    void processCVMat(const std::vector<std_msgs::Header> &headers,
                      const std::vector<cv::Mat> &grey,
                      const std::vector<cv::Mat> &imgs);
    void processBag(const std::string &fname);

    // ----------------------------------------------------------
    ros::NodeHandle                     nh_;
    unsigned int                        fnum_{0};
    rosbag::Bag                         outbag_;
    std::vector<std::string>            tagTopics_;
    std::vector<std::string>            imageTopics_;
    bool                                imagesAreCompressed_{false};
    bool                                annotateImages_{false};
    apriltag_ros::ApriltagDetector::Ptr detector_;
  };
}

#endif
