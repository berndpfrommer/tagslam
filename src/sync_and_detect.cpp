/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/sync_and_detect.h"
#include "tagslam/logging.h"

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
      } else {
        BOMB_OUT("INVALID DETECTOR TYPE: " << detectorType_);
      }
      detectors_.back()->set_black_border(borderWidth);
    }

    nh_.param<int>("max_number_frames", maxFrameNumber_, 1000000);
    nh_.param<int>("skip", skip_, 1);
    nh_.param<bool>("images_are_compressed", imagesAreCompressed_, false);
    nh_.param<bool>("annotate_images", annotateImages_, false);
    nh_.param<std::string>("bag_file", bagFile_, "");
    std::string outfname;
    nh_.param<std::string>("output_bag_file", outfname, "output.bag");
    outbag_.open(outfname, rosbag::bagmode::Write);
    if (runOnline()) {
      subscribe();
    } else {
      processBag(bagFile_);
    }
    return (true);
  }

  SyncAndDetect::Odom::Odom(
    ros::NodeHandle &nh, const std::string &topic,
    ImageOdometrySync* sync) : topic_(topic), sync_(sync) {
    sub_ = nh.subscribe(topic, 10, &Odom::callback, this);
  }

  void SyncAndDetect::Odom::callback(const OdometryConstPtr &odom) {
    sync_->process(topic_, odom);
  }

  SyncAndDetect::View::View(
    ImageTransport *it, const std::string &topic,
    ImageOdometrySync* sync, bool useCompressed) : topic_(topic), sync_(sync) {
    
    image_transport::TransportHints th(useCompressed ? "compressed" : "raw");
    sub_ = it->subscribe(topic, 10, &View::callback, this, th);
  }

  void SyncAndDetect::View::callback(const ImageConstPtr &image) {
    sync_->process(topic_, image);
  }

  void
  SyncAndDetect::subscribe() {
    std::vector<std::vector<std::string>> tpv;
    tpv.push_back(imageTopics_);
    tpv.push_back(odometryTopics_);
    imageTransport_.reset(new image_transport::ImageTransport(nh_));
    sync_.reset(new flex_sync::Sync<Image, Odometry>(
                  tpv, std::bind(&SyncAndDetect::processImages,
                                 this, std::placeholders::_1,
                                 std::placeholders::_2)));
    for (const auto &i: irange(0ul, imageTopics_.size())) {
      views_.emplace_back(new View(imageTransport_.get(),
                                   imageTopics_[i], sync_.get(),
                                   imagesAreCompressed_));
      pubs_.push_back(nh_.advertise<apriltag_msgs::ApriltagArrayStamped>(
                        tagTopics_[i], 10));
    }
    for (const auto &topic: odometryTopics_) {
      odoms_.emplace_back(new Odom(nh_, topic, sync_.get()));
    }
  }

  void
  SyncAndDetect::processCVMat(const std::vector<std_msgs::Header> &headers,
                              const std::vector<cv::Mat> &grey,
                              const std::vector<cv::Mat> &imgs) {
    int totTags(0);
    typedef std::vector<apriltag_msgs::Apriltag> TagVec;
    std::vector<TagVec> allTags(grey.size());
    if (detectorType_ == "Umich") {
      profiler_.reset("detect");
#pragma omp parallel for
      for (int i = 0; i < (int)grey.size(); i++) {
        allTags[i] = detectors_[i]->Detect(grey[i]);
      }
      profiler_.record("detect", grey.size());
    } else {
      profiler_.reset("detect");
#pragma omp parallel for
      for (int i = 0; i < (int)grey.size(); i++) {
        allTags[i] = detectors_[i]->Detect(grey[i]);
      }
      profiler_.record("detect", grey.size());
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
      iterate_through_bag<CompressedImage>(
        imageTopics_, odometryTopics_, &view, &outbag_,
        std::bind(&SyncAndDetect::processCompressedImages,
                  this, std::placeholders::_1, std::placeholders::_2));
    } else {
      iterate_through_bag<sensor_msgs::Image>(
        imageTopics_, odometryTopics_, &view, &outbag_,
        std::bind(&SyncAndDetect::processImages,
                  this, std::placeholders::_1, std::placeholders::_2));
    }
    bag.close();
    outbag_.close();
    ros::shutdown();
  }
  
}  // namespace
