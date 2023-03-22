/*-*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/sync_and_detect.h"
#include "tagslam/logging.h"

#include <apriltag_msgs/ApriltagArrayStamped.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <boost/range/irange.hpp>
#include <math.h>
#include <fstream>
#include <iomanip>
#include <functional>

namespace tagslam {
  using boost::irange;

  SyncAndDetect::SyncAndDetect(const ros::NodeHandle& pnh) :  nh_(pnh) {
  }

  SyncAndDetect::~SyncAndDetect() {
    std::cout << profiler_ << std::endl;
    for (const auto &d: detectors_) {
      d->print_profiling_info();
    }
  }

  using Family = apriltag_ros::TagFamily;
  bool SyncAndDetect::initialize() {
    int tagF;
    nh_.param<int>("tag_family", tagF, 0);
    Family tagFamily = static_cast<Family>(tagF);
    if (tagFamily != Family::tf36h11 &&
        tagFamily != Family::tf25h9  &&
        tagFamily != Family::tf16h5) {
      BOMB_OUT("invalid tag family!");
    }
    nh_.getParam("odometry_topics", odometryTopics_);
    if (!nh_.getParam("image_topics", imageTopics_)) {
      BOMB_OUT("must specify image_topics parameter!");
    }
    if (!nh_.getParam("image_output_topics", imageOutputTopics_)) {
      imageOutputTopics_ = imageTopics_;
    }
    if (!nh_.getParam("tag_topics", tagTopics_)) {
      BOMB_OUT("must specify tagTopics parameter!");
    }
    if (tagTopics_.size() != imageTopics_.size()) {
      BOMB_OUT("must have same number of tag_topics and image_topics!");
    }
    nh_.param<std::string>("detector_type", detectorType_, "Mit");
    int borderWidth;
    nh_.param<int>("black_border_width", borderWidth, 1);
    for (const auto &i: irange(0ul, imageTopics_.size())) {
      (void)i;
      if (detectorType_ == "Mit") {
        detectors_.push_back(apriltag_ros::ApriltagDetector::Create(
                              apriltag_ros::DetectorType::Mit, tagFamily));
      } else if (detectorType_ == "Umich") {
        detectors_.push_back(apriltag_ros::ApriltagDetector::Create(
                               apriltag_ros::DetectorType::Umich, tagFamily));
        const int decim = nh_.param<int>("decimate", 1);
        if (decim > 1) {
          detectors_.back()->set_decimate(decim);
        }
      } else {
        BOMB_OUT("INVALID DETECTOR TYPE: " << detectorType_);
      }
      detectors_.back()->set_black_border(borderWidth);
    }

    nh_.param<int>("max_number_frames", maxFrameNumber_, 1000000);
    nh_.param<int>("skip", skip_, 1);
    nh_.param<bool>("images_are_compressed", imagesAreCompressed_, false);
    nh_.param<bool>("annotate_images", annotateImages_, false);
    nh_.param<bool>("use_approximate_sync", useApproximateSync_, false);
    nh_.param<int>("sync_queue_size", syncQueueSize_, 100);
    int subQueueSize, pubQueueSize;
    nh_.param<int>("subscribe_queue_size", subQueueSize, 10);
    nh_.param<int>("publish_queue_size", pubQueueSize, 10);
    nh_.param<bool>("parallelize_detection", parallelizeDetection_, true);

    nh_.param<std::string>("bag_file", bagFile_, "");
    std::string outfname;
    nh_.param<std::string>("output_bag_file", outfname, "output.bag");
    outbag_.open(outfname, rosbag::bagmode::Write);
    if (runOnline()) {
      if (useApproximateSync_) {
        approxSubscriber_.reset(
          new Subscriber<ApproximateSync>(
            imageTopics_, odometryTopics_, this, nh_,
            syncQueueSize_, subQueueSize,
            imagesAreCompressed_));
      } else {
        exactSubscriber_.reset(
          new Subscriber<ExactSync>(
            imageTopics_, odometryTopics_, this, nh_,
            syncQueueSize_, subQueueSize,
            imagesAreCompressed_));
      }
      for (const auto &topic: tagTopics_) {
        pubs_.push_back(nh_.advertise<apriltag_msgs::ApriltagArrayStamped>(
                          topic, pubQueueSize));
      }
    } else {
      processBag(bagFile_);
    }
    return (true);
  }


  void
  SyncAndDetect::processCVMat(const std::vector<std_msgs::Header> &headers,
                              const std::vector<cv::Mat> &grey,
                              const std::vector<cv::Mat> &imgs) {
    int totTags(0);
    typedef std::vector<apriltag_msgs::Apriltag> TagVec;
    std::vector<TagVec> allTags(grey.size());
    profiler_.reset("detect");
    if (parallelizeDetection_) {
#pragma omp parallel for
      for (int i = 0; i < (int)grey.size(); i++) {
        allTags[i] = detectors_[i]->Detect(grey[i]);
      }
    } else {
      for (int i = 0; i < (int)grey.size(); i++) {
        allTags[i] = detectors_[i]->Detect(grey[i]);
      }
    }
    profiler_.record("detect", grey.size());

    sensor_msgs::CompressedImage msg;
    msg.format = "jpeg";
    std::vector<int> param(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 80;//default(95) 0-100

    for (const auto i: irange(0ul, grey.size())) {
      const std::vector<apriltag_msgs::Apriltag> tags = allTags[i];
      totTags += tags.size();
      apriltag_msgs::ApriltagArrayStamped tagMsg;
      tagMsg.header = headers[i];
      for (const auto &tag: tags) {
        tagMsg.apriltags.push_back(tag);
      }
      if (headers[i].stamp.toSec() != 0) {
        outbag_.write<apriltag_msgs::ApriltagArrayStamped>(
          tagTopics_[i], headers[i].stamp, tagMsg);
        if (runOnline()) {
          pubs_[i].publish(tagMsg);
        }
      }
      if (annotateImages_) {
        cv::Mat colorImg = imgs[i].clone();
        if (!tags.empty()) {
          apriltag_ros::DrawApriltags(colorImg, tags);
        }
        
        msg.header = headers[i];
        cv::imencode(".jpg", colorImg, msg.data, param);

        if(headers[i].stamp.toSec() != 0)
          outbag_.write<sensor_msgs::CompressedImage>(
            imageOutputTopics_[i], headers[i].stamp, msg);
      }
    }
    ROS_INFO_STREAM("frame " << fnum_ << " " << headers[0].stamp
                    << " detected " << totTags << " tags with "
                    << grey.size() << " cameras");
    fnum_++;
  }

  void SyncAndDetect::processCompressedImages(
    const std::vector<CompressedImageConstPtr> &msgvec,
    const std::vector<OdometryConstPtr> &odom) {
    if (msgvec.empty()) {
      ROS_WARN("got empty image vector!");
      return;
    }
    if (fnum_ % skip_ != 0) {
      ROS_INFO_STREAM("skipped frame number " << fnum_);
      fnum_++;
      return;
    }
    std::vector<cv::Mat> images, grey_images;
    std::vector<std_msgs::Header> headers;
    for (const auto i: irange(0ul, msgvec.size())) {
      const auto &img = msgvec[i];
      cv::Mat im1 = cv_bridge::toCvCopy(
        img, sensor_msgs::image_encodings::MONO8)->image;
      cv::Mat im;
      cv::cvtColor(im1, im, cv::COLOR_BayerBG2BGR);
      images.push_back(im);
      cv::Mat im_grey;
      cv::cvtColor(im, im_grey, cv::COLOR_BGR2GRAY);
      grey_images.push_back(im_grey);
      headers.push_back(img->header);
    }
    processCVMat(headers, grey_images, images);
    for (const auto i: irange(0ul, odom.size())) {
      outbag_.write<Odometry>(odometryTopics_[i],
                              odom[i]->header.stamp, odom[i]);
    }
  }

  void
  SyncAndDetect::processImages(const std::vector<ImageConstPtr> &msgvec,
                               const std::vector<OdometryConstPtr> &odom) {
    if (msgvec.empty()) {
      ROS_WARN("got empty image vector!");
      return;
    }
    if (fnum_ % skip_ != 0) {
      ROS_INFO_STREAM("skipped frame number " << fnum_);
      fnum_++;
      return;
    }
    std::vector<cv::Mat> images, grey_images;
    std::vector<std_msgs::Header> headers;
    for (const auto i: irange(0ul, msgvec.size())) {
      const auto &img = msgvec[i];
      cv::Mat im = cv_bridge::toCvCopy(
        img, sensor_msgs::image_encodings::BGR8)->image;
      images.push_back(im);
      cv::Mat im_grey = cv_bridge::toCvCopy(
        img, sensor_msgs::image_encodings::MONO8)->image;
      grey_images.push_back(im_grey);
      headers.push_back(img->header);
    }
    processCVMat(headers, grey_images, images);
        
    for (const auto i: irange(0ul, odom.size())) {
      outbag_.write<Odometry>(odometryTopics_[i],
                              odom[i]->header.stamp, odom[i]);
    }
  }

  void SyncAndDetect::processBag(const std::string &fname) {
    rosbag::Bag bag;
    bag.open(fname, rosbag::bagmode::Read);
    ROS_INFO_STREAM("playing from file: " << fname);
    double start_time(0), duration(-1);
    nh_.param<double>("start_time", start_time, 0);
    nh_.param<double>("duration", duration, -1);
    ros::Time t_start =
      rosbag::View(bag).getBeginTime() + ros::Duration(start_time);
    ros::Time t_end =
      duration >= 0 ? t_start + ros::Duration(duration) : ros::TIME_MAX;
    std::vector<std::string> allTopics = imageTopics_;
    allTopics.insert(allTopics.end(),odometryTopics_.begin(),
                     odometryTopics_.end());
    rosbag::View view(bag, rosbag::TopicQuery(allTopics), t_start, t_end);
    for (const auto i: irange(0ul, tagTopics_.size())) {
      ROS_INFO_STREAM("image topic: "  << imageTopics_[i]
                      << " maps to: " << tagTopics_[i]);
    }

    if (imagesAreCompressed_) {
      if (useApproximateSync_) {
        iterate_through_bag<ApproximateCompressedSync, CompressedImage>(
          imageTopics_, odometryTopics_, &view, &outbag_, syncQueueSize_,
          std::bind(&SyncAndDetect::processCompressedImages,
                    this, std::placeholders::_1, std::placeholders::_2));
      } else {
        iterate_through_bag<ExactCompressedSync, CompressedImage>(
          imageTopics_, odometryTopics_, &view, &outbag_,  syncQueueSize_,
          std::bind(&SyncAndDetect::processCompressedImages,
                    this, std::placeholders::_1, std::placeholders::_2));
      }
    } else {
      if (useApproximateSync_) {
        iterate_through_bag<ApproximateSync, sensor_msgs::Image>(
          imageTopics_, odometryTopics_, &view, &outbag_,  syncQueueSize_,
          std::bind(&SyncAndDetect::processImages,
                    this, std::placeholders::_1, std::placeholders::_2));
      } else {
        iterate_through_bag<ExactSync, sensor_msgs::Image>(
          imageTopics_, odometryTopics_, &view, &outbag_,  syncQueueSize_,
          std::bind(&SyncAndDetect::processImages,
                    this, std::placeholders::_1, std::placeholders::_2));
      }
    }
    bag.close();
    outbag_.close();
    ros::shutdown();
  }
  
}  // namespace
