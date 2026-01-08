// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TAGSLAM__SYNC_AND_DETECT_HPP_
#define TAGSLAM__SYNC_AND_DETECT_HPP_

#include <yaml-cpp/yaml.h>

#include <apriltag_detector/detector.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <flex_sync/approximate_sync.hpp>
#include <flex_sync/exact_sync.hpp>
#include <flex_sync/live_sync.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <set>

namespace flex_sync
{
using Image = sensor_msgs::msg::Image;
/*
* Specialize the Subscriber template in the flex_sync namespace to use
* the image_transport for subscribing instead of the regular transport.
*/
template <typename SyncT>
class Subscriber<SyncT, Image>
{
public:
  using TConstSharedPtr = typename Image::ConstSharedPtr;

  Subscriber(
    const std::string & topic, rclcpp::Node * node, const rclcpp::QoS & qos,
    const std::shared_ptr<SyncT> & sync)
  : topic_(topic), sync_(sync)
  {
    const std::string param_name = topic + ".image_transport";
    sub_ = std::make_shared<image_transport::Subscriber>(
      image_transport::create_subscription(
#ifdef IMAGE_TRANSPORT_USE_NODEINTERFACE
        *node,
#else
        node,
#endif
        topic,
        std::bind(
          &Subscriber<SyncT, Image>::callback, this, std::placeholders::_1),
        node->get_parameter_or<std::string>(param_name, "raw"),
#ifdef IMAGE_TRANSPORT_USE_QOS
        qos
#else
        qos.get_rmw_qos_profile()
#endif
        ));
  }
  void callback(const TConstSharedPtr & msg) { sync_->process(topic_, msg); }

private:
  std::string topic_;
  std::shared_ptr<SyncT> sync_;
  std::shared_ptr<image_transport::Subscriber> sub_;
};
}  // namespace flex_sync

namespace tagslam
{
using svec = std::vector<std::string>;
using svecvec = std::vector<std::vector<std::string>>;
using Image = sensor_msgs::msg::Image;
using VecImagePtr = std::vector<Image::ConstSharedPtr>;
using ApriltagArray = apriltag_msgs::msg::AprilTagDetectionArray;
using VecApriltagArrayPtr = std::vector<ApriltagArray::ConstSharedPtr>;
using Odometry = nav_msgs::msg::Odometry;
using VecOdometryPtr = std::vector<Odometry::ConstSharedPtr>;

class SyncAndDetectListener
{
public:
  virtual ~SyncAndDetectListener() {}
  virtual void callbackTags(const VecApriltagArrayPtr & tagMsgs) = 0;
  virtual void callbackTagsAndOdom(
    const VecApriltagArrayPtr & tagMsgs, const VecOdometryPtr & odoms) = 0;
};

class Publisher : public SyncAndDetectListener
{
public:
  Publisher(
    rclcpp::Node * node, const svec & tag_topics, const svec & odom_topics);
  void callbackTags(const VecApriltagArrayPtr & tagMsgs) final;
  void callbackTagsAndOdom(
    const VecApriltagArrayPtr & tagMsgs, const VecOdometryPtr & odoms) final;

private:
  void publishTags(const VecApriltagArrayPtr & tagMsgs);
  // ---------------- variables -------------------
  std::vector<rclcpp::Publisher<ApriltagArray>::SharedPtr> tag_pub_;
  std::vector<rclcpp::Publisher<Odometry>::SharedPtr> odom_pub_;
};

class SyncAndDetect : public rclcpp::Node
{
public:
  explicit SyncAndDetect(const rclcpp::NodeOptions & opt);
  ~SyncAndDetect();
  void setListener(const std::shared_ptr<SyncAndDetectListener> & m);
  const std::vector<std::pair<std::string, std::string>> & getImageTopics()
    const
  {
    return (image_topics_);
  }
  const svec & getOdomTopics() const { return (odom_topics_); }
  const svec & getSyncedOdomTopics() const { return (synced_odom_topics_); }
  const svec & getTagTopics() const { return (tag_topics_); }
  const auto & getNumberOfFrames() const { return (num_frames_); }

private:
  using ImageExactSync = flex_sync::LiveSync<flex_sync::ExactSync<Image>>;
  using ImageAndOdomExactSync =
    flex_sync::LiveSync<flex_sync::ExactSync<Image, Odometry>>;
  using ImageApproxSync =
    flex_sync::LiveSync<flex_sync::ApproximateSync<Image>>;
  using ImageAndOdomApproxSync =
    flex_sync::LiveSync<flex_sync::ApproximateSync<Image, Odometry>>;

  void getTopics();
  void parseCameras(const YAML::Node & conf);
  void parseBodies(const YAML::Node & conf);

  YAML::Node loadFileFromParameter(const std::string & name);
  void callbackImageAndOdom(
    const VecImagePtr & imgs, const VecOdometryPtr & odoms);
  void callbackImage(const VecImagePtr & imgs);
  size_t getNumberOfTagsDetected() const;

  void subscribe(
    const std::vector<std::pair<std::string, std::string>> & img_topics,
    const svec & odom_topics, const svec & detectors);

  size_t tagsFromImages(
    const VecImagePtr & imgs, VecApriltagArrayPtr * tagMsgs);

  // ----------------- variables ----------------------------------
  pluginlib::ClassLoader<apriltag_detector::Detector> detector_loader_;
  std::vector<std::shared_ptr<apriltag_detector::Detector>> detectors_;
  size_t num_tags_detected_{0};
  size_t num_frames_{0};
  svec odom_topics_;
  svec synced_odom_topics_;
  std::vector<std::pair<std::string, std::string>> image_topics_;
  svec tag_topics_;
  svec detector_names_;
  std::set<std::string> detector_types_;
  std::shared_ptr<SyncAndDetectListener> listener_;
  std::shared_ptr<Publisher> pub_;
  std::shared_ptr<ImageExactSync> image_exact_sync_;
  std::shared_ptr<ImageAndOdomExactSync> image_odom_exact_sync_;
  std::shared_ptr<ImageApproxSync> image_approx_sync_;
  std::shared_ptr<ImageAndOdomApproxSync> image_odom_approx_sync_;
};

}  // namespace tagslam

#endif  // TAGSLAM__SYNC_AND_DETECT_HPP_
