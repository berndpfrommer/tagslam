/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/profiler.h"

#include <flex_sync/sync.h>
#include <image_transport/image_transport.h>
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
  using ImageTransport = image_transport::ImageTransport;
  typedef flex_sync::Sync<Image, Odometry> ImageOdometrySync;

  class SyncAndDetect {
  public:
    SyncAndDetect(const ros::NodeHandle &pnh);
    ~SyncAndDetect();

    SyncAndDetect(const SyncAndDetect&) = delete;
    SyncAndDetect& operator=(const SyncAndDetect&) = delete;

    bool initialize();

  private:
    class View {
    public:
      View(ImageTransport *it, const std::string &topic,
           ImageOdometrySync* sync, bool useCompressed);
    private:
      std::string       topic_;
      ImageOdometrySync *sync_;
      image_transport::Subscriber sub_;
      void callback(const ImageConstPtr &image);
    };

    class Odom {
    public:
      Odom(ros::NodeHandle &nh, const std::string &topic,
           ImageOdometrySync *sync);
    private:
      std::string       topic_;
      ImageOdometrySync *sync_;
      ros::Subscriber   sub_;
      void callback(const OdometryConstPtr &odom);
    };

    void subscribe();
    bool runOnline() const { return (bagFile_.empty()); }
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
    std::string                         bagFile_;
    rosbag::Bag                         outbag_;
    std::vector<std::string>            tagTopics_;
    std::vector<std::string>            imageTopics_;
    std::vector<std::string>            odometryTopics_;
    std::vector<std::string>            imageOutputTopics_;
    bool                                imagesAreCompressed_{false};
    bool                                annotateImages_{false};
    int                                 maxFrameNumber_;
    int                                 skip_{1};
    std::vector<apriltag_ros::ApriltagDetector::Ptr> detectors_;
    std::string                         detectorType_;
    Profiler                            profiler_;
    std::shared_ptr<ImageTransport>     imageTransport_;
    std::shared_ptr<ImageOdometrySync>  sync_;
    std::vector<std::shared_ptr<View>>  views_;
    std::vector<std::shared_ptr<Odom>>  odoms_;
    std::vector<ros::Publisher>         pubs_;
  };
}
