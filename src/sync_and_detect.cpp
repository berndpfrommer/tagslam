/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/sync_and_detect.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <boost/range/irange.hpp>
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <math.h>
#include <fstream>
#include <iomanip>
#include <functional>

namespace tagslam {
  using boost::irange;

  SyncAndDetect::SyncAndDetect(const ros::NodeHandle& pnh) :  nh_(pnh) {
  }

  SyncAndDetect::~SyncAndDetect() {
  }

  bool SyncAndDetect::initialize() {
    if (!nh_.getParam("image_topics", imageTopics_)) {
      ROS_ERROR("must specify image_topics parameter!");
      return (false);
    }
    if (!nh_.getParam("image_output_topics", imageOutputTopics_)) {
      imageOutputTopics_ = imageTopics_;
    }
    if (!nh_.getParam("tag_topics", tagTopics_)) {
      ROS_ERROR("must specify tagTopics parameter!");
      return (false);
    }
    if (tagTopics_.size() != imageTopics_.size()) {
      ROS_ERROR("must have same number of tag_topics and image_topics!");
      return (false);
    }
    nh_.param<std::string>("detector_type", detectorType_, "Mit");
    if (detectorType_ == "Mit") {
      detector_ = apriltag_ros::ApriltagDetector::Create(
        apriltag_ros::DetectorType::Mit, apriltag_ros::TagFamily::tf36h11);
    } else if (detectorType_ == "Umich") {
      detector_ = apriltag_ros::ApriltagDetector::Create(
        apriltag_ros::DetectorType::Umich, apriltag_ros::TagFamily::tf36h11);
    } else {
      ROS_ERROR_STREAM("INVALID DETECTOR TYPE: " << detectorType_);
    }
    int borderWidth;
    nh_.param<int>("black_border_width", borderWidth, 1);
    detector_->set_black_border(borderWidth);

    nh_.param<int>("max_number_frames", maxFrameNumber_, 1000000);
    nh_.param<bool>("images_are_compressed", imagesAreCompressed_, false);
    nh_.param<bool>("annotate_images", annotateImages_, false);
    std::string bagFile;
    nh_.param<std::string>("bag_file", bagFile, "");
    std::string outfname;
    nh_.param<std::string>("output_bag_file", outfname, "output.bag");
    outbag_.open(outfname, rosbag::bagmode::Write);
    if (!bagFile.empty()) {
      processBag(bagFile);
    } else {
      ROS_ERROR("must specify bag_file parameter!");
      outbag_.close();
      return (false);
    }
    outbag_.close();
    return (true);
  }


  void SyncAndDetect::processCVMat(const std::vector<std_msgs::Header> &headers,
                                   const std::vector<cv::Mat> &grey,
                                   const std::vector<cv::Mat> &imgs) {
    int totTags(0);
    typedef std::vector<apriltag_msgs::Apriltag> TagVec;
    std::vector<TagVec> allTags(grey.size());
    if (detectorType_ == "Umich") {
      for (int i = 0; i < (int)grey.size(); i++) {
        allTags[i] = detector_->Detect(grey[i]);
      }
    } else {
#pragma omp parallel for
      for (int i = 0; i < (int)grey.size(); i++) {
        allTags[i] = detector_->Detect(grey[i]);
      }
    }

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
      if(headers[i].stamp.toSec() != 0)
        outbag_.write<apriltag_msgs::ApriltagArrayStamped>(tagTopics_[i], headers[i].stamp, tagMsg);
      if (annotateImages_) {
        cv::Mat colorImg = imgs[i].clone();
        if (!tags.empty()) {
          apriltag_ros::DrawApriltags(colorImg, tags);
        }
        
        msg.header = headers[i];
        cv::imencode(".jpg", colorImg, msg.data, param);

        if(headers[i].stamp.toSec() != 0)
          outbag_.write<sensor_msgs::CompressedImage>(imageOutputTopics_[i], headers[i].stamp, msg);
      }
    }
    ROS_INFO_STREAM("frame " << fnum_ << " " << headers[0].stamp << " detected "
                    << totTags << " tags with " << grey.size() << " cameras");
    fnum_++;
  }

  void SyncAndDetect::processCompressedImages(const std::vector<CompressedImageConstPtr> &msgvec) {
    if (msgvec.empty()) {
      ROS_ERROR("got empty image vector!");
      return;
    }
    std::vector<cv::Mat> images, grey_images;
    std::vector<std_msgs::Header> headers;
    for (const auto i: irange(0ul, msgvec.size())) {
      const auto &img = msgvec[i];
      cv::Mat im1 = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8)->image;
      cv::Mat im;
      cv::cvtColor(im1, im, cv::COLOR_BayerBG2BGR);
      images.push_back(im);
      cv::Mat im_grey;
      cv::cvtColor(im, im_grey, cv::COLOR_BGR2GRAY);
      grey_images.push_back(im_grey);
      headers.push_back(img->header);
    }
    processCVMat(headers, grey_images, images);
  }

  void SyncAndDetect::processImages(const std::vector<ImageConstPtr> &msgvec) {
    if (msgvec.empty()) {
      ROS_ERROR("got empty image vector!");
      return;
    }
    std::vector<cv::Mat> images, grey_images;
    std::vector<std_msgs::Header> headers;
    for (const auto i: irange(0ul, msgvec.size())) {
      const auto &img = msgvec[i];
      cv::Mat im = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8)->image;
      images.push_back(im);
      cv::Mat im_grey = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8)->image;
      grey_images.push_back(im_grey);
      headers.push_back(img->header);
    }
    processCVMat(headers, grey_images, images);
  }

  void SyncAndDetect::processBag(const std::string &fname) {
    rosbag::Bag bag;
    bag.open(fname, rosbag::bagmode::Read);
    ROS_INFO_STREAM("playing from file: " << fname);
    double start_time(0);
    nh_.param<double>("start_time", start_time, 0);
    ros::Time t_start(start_time);
    rosbag::View view(bag, rosbag::TopicQuery(imageTopics_));
    rosbag::View t0View(bag);
    for (const auto i: irange(0ul, tagTopics_.size())) {
      ROS_INFO_STREAM("image topic: "  << imageTopics_[i] << " maps to: " << tagTopics_[i]);
    }

    if (imagesAreCompressed_) {
      iterate_through_bag<CompressedImage>(imageTopics_, &view, &outbag_,
                                           std::bind(&SyncAndDetect::processCompressedImages,
                                                     this, std::placeholders::_1));
    } else {
      iterate_through_bag<sensor_msgs::Image>(imageTopics_,&view, &outbag_,
                                              std::bind(&SyncAndDetect::processImages,
                                                        this, std::placeholders::_1));
    }
    bag.close();
    ros::shutdown();
  }
  
}  // namespace
