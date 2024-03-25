/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <tagslam/profiler.hpp>
ag_detector.h >
#include <flex_sync/approximate_sync.h>
#include <flex_sync/exact_sync.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <memory>
#include <string>
#include <tagslam/profiler.hpp>
#include <vector>

  namespace tagslam
{
  using Image = sensor_msgs::Image;
  using ImageConstPtr = sensor_msgs::ImageConstPtr;
  using CompressedImage = sensor_msgs::CompressedImage;
  using CompressedImageConstPtr = sensor_msgs::CompressedImageConstPtr;
  using Odometry = nav_msgs::Odometry;
  using OdometryConstPtr = nav_msgs::OdometryConstPtr;
  using ImageTransport = image_transport::ImageTransport;

  class SyncAndDetect
  {
  public:
    SyncAndDetect(const ros::NodeHandle & pnh);
    ~SyncAndDetect();

    SyncAndDetect(const SyncAndDetect &) = delete;
    SyncAndDetect & operator=(const SyncAndDetect &) = delete;

    bool initialize();

  private:
    typedef flex_sync::ExactSync<Image, Odometry> ExactSync;
    typedef flex_sync::ApproximateSync<Image, Odometry> ApproximateSync;
    typedef flex_sync::ExactSync<CompressedImage, Odometry> ExactCompressedSync;
    typedef flex_sync::ApproximateSync<CompressedImage, Odometry>
      ApproximateCompressedSync;

    template <class SyncT>
    class Subscriber
    {
    public:
      //
      // the subscriber class deals with live ros node subscriptions
      //
      Subscriber(
        const std::vector<std::string> & imageTopics,
        const std::vector<std::string> & odomTopics, SyncAndDetect * sand,
        ros::NodeHandle & nh, int syncQueueSize, int subQueueSize,
        bool imagesAreCompressed)
      {
        const std::vector<std::vector<std::string>> tpv = {
          imageTopics, odomTopics};
        auto cb = std::bind(
          &SyncAndDetect::processImages, sand, std::placeholders::_1,
          std::placeholders::_2);
        sync_.reset(new SyncT(tpv, cb, syncQueueSize));
        imageTransport_.reset(new image_transport::ImageTransport(nh));
        for (size_t i = 0; i < imageTopics.size(); i++) {
          views_.emplace_back(new View<SyncT>(
            imageTransport_.get(), imageTopics[i], sync_.get(), subQueueSize,
            imagesAreCompressed));
        }
        for (const auto & topic : odomTopics) {
          odoms_.emplace_back(
            new Odom<SyncT>(nh, topic, sync_.get(), subQueueSize));
        }
      }

    private:
      template <class VSyncT>
      class View
      {
      public:
        View(
          ImageTransport * it, const std::string & topic, VSyncT * sync, int qs,
          bool useCompressed)
        : topic_(topic), sync_(sync)
        {
          image_transport::TransportHints th(
            useCompressed ? "compressed" : "raw");
          sub_ = it->subscribe(topic, qs, &View::callback, this, th);
        }

      private:
        void callback(const ImageConstPtr & image)
        {
          sync_->process(topic_, image);
        }
        std::string topic_;
        VSyncT * sync_;
        image_transport::Subscriber sub_;
      };

      template <class OSyncT>
      class Odom
      {
      public:
        Odom(
          ros::NodeHandle & nh, const std::string & topic, OSyncT * sync,
          int qs)
        : topic_(topic), sync_(sync)
        {
          sub_ = nh.subscribe(topic, qs, &Odom::callback, this);
        }

      private:
        void callback(const OdometryConstPtr & odom)
        {
          sync_->process(topic_, odom);
        }
        std::string topic_;
        OSyncT * sync_;
        ros::Subscriber sub_;
      };
      // ------ variables
      std::shared_ptr<ImageTransport> imageTransport_;
      std::shared_ptr<SyncT> sync_;
      std::vector<std::shared_ptr<View<SyncT>>> views_;
      std::vector<std::shared_ptr<Odom<SyncT>>> odoms_;
    };

    bool runOnline() const { return (bagFile_.empty()); }
    void processImages(
      const std::vector<ImageConstPtr> & msgvec,
      const std::vector<OdometryConstPtr> & odomvec);
    void processCompressedImages(
      const std::vector<CompressedImageConstPtr> & mv,
      const std::vector<OdometryConstPtr> & odomvec);
    void processCVMat(
      const std::vector<std_msgs::Header> & headers,
      const std::vector<cv::Mat> & grey, const std::vector<cv::Mat> & imgs);
    void processBag(const std::string & fname);
    template <typename SyncT, typename T>
    void iterate_through_bag(
      const std::vector<std::string> & topics,
      const std::vector<std::string> & odomTopics, rosbag::View * view,
      rosbag::Bag * bag, size_t qs,
      const std::function<void(
        const std::vector<std::shared_ptr<T const>> &,
        const std::vector<OdometryConstPtr> &)> & cb)
    {
      std::vector<std::vector<std::string>> tpv;
      tpv.push_back(topics);
      tpv.push_back(odomTopics);
      SyncT sync(tpv, cb, qs);
      for (const rosbag::MessageInstance & m : *view) {
        OdometryConstPtr odom = m.instantiate<Odometry>();
        if (odom) {
          sync.process(m.getTopic(), odom);
        } else {
          std::shared_ptr<T const> img = m.instantiate<T>();
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
    ros::NodeHandle nh_;
    unsigned int fnum_{0};
    std::string bagFile_;
    rosbag::Bag outbag_;
    std::vector<std::string> imageTopics_;
    std::vector<std::string> odometryTopics_;
    std::vector<std::string> imageOutputTopics_;
    std::vector<std::string> tagTopics_;
    bool imagesAreCompressed_{false};
    bool annotateImages_{false};
    bool useApproximateSync_{false};
    int syncQueueSize_;
    int maxFrameNumber_;
    int skip_{1};
    std::vector<apriltag_ros::ApriltagDetector::Ptr> detectors_;
    std::string detectorType_;
    Profiler profiler_;
    std::shared_ptr<Subscriber<ApproximateSync>> approxSubscriber_;
    std::shared_ptr<Subscriber<ExactSync>> exactSubscriber_;
    std::vector<ros::Publisher> pubs_;
    bool parallelizeDetection_{true};
  };
}  // namespace tagslam
