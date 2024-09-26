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

#include <algorithm>
#ifdef USE_CV_BRIDGE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <tagslam/logging.hpp>
#include <tagslam/sync_and_detect.hpp>

namespace tagslam
{
using std::string;
using std::placeholders::_1;
using std::placeholders::_2;

Publisher::Publisher(
  rclcpp::Node * node, const svec & tag_topics, const svec & odom_topics)
{
  for (const auto & tt : tag_topics) {
    tag_pub_.push_back(node->create_publisher<ApriltagArray>(tt, 10));
  }
  for (const auto & ot : odom_topics) {
    odom_pub_.push_back(node->create_publisher<Odometry>(ot, 10));
  }
}

void Publisher::callbackTags(const VecApriltagArrayPtr & tagMsgs)
{
  publishTags(tagMsgs);
}

void Publisher::callbackTagsAndOdom(
  const VecApriltagArrayPtr & tagMsgs, const VecOdometryPtr & odoms)
{
  publishTags(tagMsgs);
  for (size_t i = 0; i < odoms.size(); i++) {
    odom_pub_[i]->publish(*odoms[i]);
  }
}

void Publisher::publishTags(const VecApriltagArrayPtr & tagMsgs)
{
  for (size_t i = 0; i < tagMsgs.size(); i++) {
    tag_pub_[i]->publish(*tagMsgs[i]);
  }
}

SyncAndDetect::SyncAndDetect(const rclcpp::NodeOptions & opt)
: Node("sync_and_detect", opt),
  detector_loader_("apriltag_detector", "apriltag_detector::Detector")
{
  getTopics();
  if (declare_parameter<bool>("publish", true)) {
    pub_ = std::make_shared<Publisher>(this, tag_topics_, synced_odom_topics_);
    listener_ = pub_;
  }
  subscribe(image_topics_, odom_topics_, detector_names_);
}

SyncAndDetect::~SyncAndDetect()
{
  image_exact_sync_.reset();
  image_approx_sync_.reset();
  image_odom_exact_sync_.reset();
  image_odom_approx_sync_.reset();
  detectors_.clear();
  pub_.reset();
  for (const auto & type : detector_types_) {
    detector_loader_.unloadLibraryForClass(
      "apriltag_detector_" + type + "::Detector");
  }
}

void SyncAndDetect::setListener(
  const std::shared_ptr<SyncAndDetectListener> & m)
{
  listener_ = m;
}

void SyncAndDetect::getTopics()
{
  parseCameras(loadFileFromParameter("cameras"));
  auto conf = loadFileFromParameter("tagslam_config");
  if (conf["bodies"]) {
    parseBodies(conf["bodies"]);
  }
}

void SyncAndDetect::parseCameras(const YAML::Node & conf)
{
  svec cam_names;
  for (const auto & c : conf) {
    const auto name = c.first.as<string>();
    if (name.find("cam") != string::npos) {
      cam_names.push_back(name);
    }
  }
  std::sort(cam_names.begin(), cam_names.end(), [](string a, string b) {
    return a < b;
  });
  for (const auto & cam : cam_names) {
    const auto & c = conf[cam];
    if (!c["image_topic"] || !c["tag_topic"]) {
      BOMB_OUT("must specify image_topic and tag_topic for camera " << cam);
    }
    tag_topics_.push_back(c["tag_topic"].as<string>());
    image_topics_.push_back(
      {c["image_topic"].as<string>(),
       c["image_transport"] ? c["image_transport"].as<string>() : "raw"});
    detector_names_.push_back(
      c["tag_detector"] ? c["tag_detector"].as<string>() : "umich");
    LOG_INFO(
      "camera " << cam << " topic: " << image_topics_.back().first
                << " transp: " << image_topics_.back().second
                << " detect: " << detector_names_.back());
  }
}

void SyncAndDetect::parseBodies(const YAML::Node & conf)
{
  for (const auto & body : conf) {
    if (body.IsMap()) {
      for (const auto & bm : body) {
        auto node = bm.second;
        if (node["odom_topic"]) {
          const auto topic = node["odom_topic"].as<std::string>();
          odom_topics_.push_back(topic);
          synced_odom_topics_.push_back(topic + "_synced");
          LOG_INFO(
            "body " << bm.first.as<std::string>()
                    << " has odom topic: " << topic);
        }
      }
    }
  }
}

YAML::Node SyncAndDetect::loadFileFromParameter(const string & name)
{
  const string p = declare_parameter<string>(name, "");
  if (p.empty()) {
    BOMB_OUT("parameter " << name << " must be specified and non-empty!");
  }
  YAML::Node config = YAML::LoadFile(p);
  if (config.IsNull()) {
    BOMB_OUT("cannot open config file: " << p);
  }
  return (config);
}

void SyncAndDetect::callbackImageAndOdom(
  const VecImagePtr & imgs, const VecOdometryPtr & odoms)
{
  VecApriltagArrayPtr tagMsgs;
  num_tags_detected_ += tagsFromImages(imgs, &tagMsgs);
  num_frames_++;
  if (num_frames_ % 10 == 0) {
    LOG_INFO("frame " << num_frames_ << " total tags: " << num_tags_detected_);
  }
  if (listener_) {
    listener_->callbackTagsAndOdom(tagMsgs, odoms);
  }
}

void SyncAndDetect::callbackImage(const VecImagePtr & imgs)
{
  VecApriltagArrayPtr tagMsgs;
  num_tags_detected_ += tagsFromImages(imgs, &tagMsgs);
  num_frames_++;
  if (num_frames_ % 10 == 0) {
    LOG_INFO("frame " << num_frames_ << " total tags: " << num_tags_detected_);
  }
  if (listener_) {
    listener_->callbackTags(tagMsgs);
  }
}

size_t SyncAndDetect::getNumberOfTagsDetected() const
{
  return (num_tags_detected_);
}
void SyncAndDetect::subscribe(
  const std::vector<std::pair<std::string, std::string>> & img_topics,
  const svec & odom_topics, const svec & detectors)
{
  svec imgs;
  for (const auto & img : img_topics) {
    imgs.push_back(img.first);
  }
  assert(img_topics.size() == detectors.size());

  for (size_t i = 0; i < img_topics.size(); i++) {
    const auto & topic = img_topics[i].first;
    declare_parameter<string>(topic + ".image_transport", img_topics[i].second);
    detectors_.push_back(detector_loader_.createSharedInstance(
      "apriltag_detector_" + detectors[i] + "::Detector"));
    detector_types_.insert(detectors[i]);
  }
  const bool use_approx_sync =
    declare_parameter<bool>("use_approximate_sync", true);
  LOG_INFO("using approximate sync: " << (use_approx_sync ? "YES" : "NO"));
  const int qs = 10;  // queue size
  if (odom_topics.empty()) {
    if (use_approx_sync) {
      image_approx_sync_ = std::make_shared<ImageApproxSync>(
        this, svecvec({imgs}),
        std::bind(&SyncAndDetect::callbackImage, this, _1), qs);
    } else {
      image_exact_sync_ = std::make_shared<ImageExactSync>(
        this, svecvec({imgs}),
        std::bind(&SyncAndDetect::callbackImage, this, _1), qs);
    }
  } else {
    if (use_approx_sync) {
      image_odom_approx_sync_ = std::make_shared<ImageAndOdomApproxSync>(
        this, svecvec({imgs, odom_topics}),
        std::bind(&SyncAndDetect::callbackImageAndOdom, this, _1, _2), qs);
    } else {
      image_odom_exact_sync_ = std::make_shared<ImageAndOdomExactSync>(
        this, svecvec({imgs, odom_topics}),
        std::bind(&SyncAndDetect::callbackImageAndOdom, this, _1, _2), qs);
    }
  }
}

size_t SyncAndDetect::tagsFromImages(
  const VecImagePtr & imgs, VecApriltagArrayPtr * tagMsgs)
{
  assert(detectors_.size() == imgs.size());
  size_t num_tags{0};
  for (size_t i = 0; i < imgs.size(); i++) {
    const auto & img = imgs[i];
    auto tags = std::make_shared<ApriltagArray>();
    tagMsgs->push_back(tags);
    tags->header = img->header;
    cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(img, "mono8");
    if (!cvImg) {
      BOMB_OUT("cannot convert image to mono!");
    }
    detectors_[i]->detect(cvImg->image, tags.get());
    num_tags += tags->detections.size();
  }
  return (num_tags);
}
}  // namespace tagslam
