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

#ifndef TAGSLAM__TAGSLAM_HPP_
#define TAGSLAM__TAGSLAM_HPP_

#if __has_include(<tf2_ros/transform_broadcaster.hpp>)
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/transform_broadcaster.h>
#endif
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <flex_sync/approximate_sync.hpp>
#include <flex_sync/exact_sync.hpp>
#include <flex_sync/live_sync.hpp>
#include <fstream>
#include <map>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <set>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tagslam/camera.hpp>
#include <tagslam/graph.hpp>
#include <tagslam/graph_updater.hpp>
#include <tagslam/measurements/measurements.hpp>
#include <tagslam/odometry_processor.hpp>
#include <tagslam/pose_with_noise.hpp>
#include <tagslam/profiler.hpp>
#include <tagslam/tag_factory.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <unordered_map>

namespace YAML
{
class Node;  // forward decl
}

namespace tagslam
{
class Body;  // forward decl
class TagSLAM : public TagFactory, public rclcpp::Node
{
  using string = std::string;
  using Apriltag = apriltag_msgs::msg::AprilTagDetection;
  using TagArray = apriltag_msgs::msg::AprilTagDetectionArray;
  using TagArrayPtr = TagArray::SharedPtr;
  using TagArrayConstPtr = TagArray::ConstSharedPtr;
  using Odometry = nav_msgs::msg::Odometry;
  using OdometryConstPtr = Odometry::ConstSharedPtr;
  using Image = sensor_msgs::msg::Image;
  using ImageConstPtr = Image::ConstSharedPtr;
  using TFMessage = tf2_msgs::msg::TFMessage;
  using Trigger = std_srvs::srv::Trigger;
  using Point = apriltag_msgs::msg::Point;
  using Path = nav_msgs::msg::Path;
  using Header = std_msgs::msg::Header;

  using ExactSync = flex_sync::ExactSync<TagArray, Odometry>;
  using ApproxSync = flex_sync::ApproximateSync<TagArray, Odometry>;
  using LiveExactSync = flex_sync::LiveSync<ExactSync>;
  using LiveApproxSync = flex_sync::LiveSync<ApproxSync>;

  using PoseCacheMap = std::map<
    string, PoseWithNoise, std::less<string>,
    Eigen::aligned_allocator<std::pair<string, PoseWithNoise>>>;

public:
  explicit TagSLAM(const rclcpp::NodeOptions & options);
  TagSLAM(const TagSLAM &) = delete;
  TagSLAM & operator=(const TagSLAM &) = delete;
  // inherited from TagFactory
  TagConstPtr findTag(int tagId) final;

  // ------ own methods
  std::vector<std::string> getTagTopics() const;
  std::vector<std::string> getOdomTopics() const;
  std::vector<std::string> getPublishedTopics() const;
  std::vector<std::pair<std::string, std::string>> getImageTopics() const;
  size_t getNumberOfFrames() const { return (static_cast<size_t>(frameNum_)); }
  void finalize(bool optimize = true);

private:
  struct ReMap
  {
    ReMap(int i, const string & cam, uint64_t ts, uint64_t te)
    : remappedId(i), camera(cam), startTime(ts), endTime(te)
    {
    }
    int remappedId;
    string camera;
    uint64_t startTime;
    uint64_t endTime;
  };
  typedef std::unordered_map<int, TagConstPtr> TagMap;
  // ---------- methods
  bool initialize();
  TagPtr addTag(int tagId, const std::shared_ptr<Body> & body) const;
  void subscribe();
  void syncCallback(
    const std::vector<TagArrayConstPtr> & msgvec1,
    const std::vector<OdometryConstPtr> & msgvec3);
  void testForOldLaunchParameters();
  void readParams();
  void readBodies(const YAML::Node & config);
  void readGlobalParameters(const YAML::Node & config);
  void readCameras(const YAML::Node & config);
  void readCameraPoses(const YAML::Node & config);
  void readDistanceMeasurements();
  void readRemap(const YAML::Node & config);
  void readSquash(const YAML::Node & config);
  void parseTimeSquash(
    const YAML::Node & sq, uint64_t t, const std::set<int> & tags);
  void fakeOdom(uint64_t tCurr, std::vector<VertexDesc> * factors);

  void processOdom(
    uint64_t t, const std::vector<OdometryConstPtr> & odomMsg,
    std::vector<VertexDesc> * factors);
  std::vector<std::vector<string>> makeTopics() const;

  void setupOdom(const std::vector<OdometryConstPtr> & odomMsgs);
  void processTagsAndOdom(
    const std::vector<TagArrayConstPtr> & tagmsgs,
    const std::vector<OdometryConstPtr> & odommsgs);

  void publishTagAndBodyTransforms(uint64_t t, TFMessage * tfMsg);
  void publishOriginalTagTransforms(uint64_t t, TFMessage * tfMsg);
  void publishCameraTransforms(uint64_t t, TFMessage * tfMsg);

  void publishTransforms(uint64_t t, bool orig = false);
  void publishBodyOdom(uint64_t t);
  void processTags(
    uint64_t t, const std::vector<TagArrayConstPtr> & tagMsgs,
    std::vector<VertexDesc> * factors);
  std::vector<TagConstPtr> findTags(const std::vector<Apriltag> & ta);
  bool anyTagsVisible(const std::vector<TagArrayConstPtr> & tagmsgs);
  void publishAll(uint64_t t);
  void plot(
    const std::shared_ptr<Trigger::Request> req,
    const std::shared_ptr<Trigger::Response> res);
  void replay(
    const std::shared_ptr<Trigger::Request> req,
    const std::shared_ptr<Trigger::Response> res);
  void dump(
    const std::shared_ptr<Trigger::Request> req,
    const std::shared_ptr<Trigger::Response> res);

  void doDump(bool optimize);
  void writeCameraPoses(const string & fname);
  void writeFullCalibration(const string & fname) const;
  void writePoses(const string & fname);
  void writeTagDiagnostics(const string & fname) const;
  void writeTimeDiagnostics(const string & fname) const;
  void writeDistanceDiagnostics(const string & fname) const;
  void writeErrorMap(const string & fname) const;
  void writeTagCorners(
    uint64_t t, int camIdx, const TagConstPtr & tag, const Point * img_corners);

  void remapAndSquash(
    uint64_t t, std::vector<TagArrayConstPtr> * remapped,
    const std::vector<TagArrayConstPtr> & orig);
  void applyDistanceMeasurements();
  void doReplay(double rate);
  void copyPosesAndReset();
  PoseWithNoise getOptimizedPoseWithNoise(const string & name);
  void openOutputBag(const string & bag_name);
  rclcpp::Time rosTime(uint64_t t) const
  {
    return (rclcpp::Time(t, this->get_clock()->get_clock_type()));
  }
  // ------ variables --------

  rclcpp::Node * node_{nullptr};
  TagMap tagMap_;
  std::shared_ptr<Body> defaultBody_;
  bool warnIgnoreTags_{false};
  GraphPtr graph_;

  GraphPtr initialGraph_;
  GraphUpdater graphUpdater_;
  CameraVec cameras_;
  BodyVec bodies_;
  BodyVec nonstaticBodies_;
  rclcpp::Publisher<Header>::SharedPtr ackPub_;
  std::vector<rclcpp::Publisher<Odometry>::SharedPtr> odomPub_;
  std::vector<rclcpp::Publisher<Path>::SharedPtr> pathPub_;
  std::vector<Path> trajectory_;
  std::unique_ptr<rosbag2_cpp::Writer> outputBag_;
  string fixedFrame_;
  bool useApproximateSync_;
  bool useFakeOdom_{false};
  bool publishAck_{false};
  bool amnesia_{false};
  int frameNum_{0};
  int maxFrameNum_{1000000};
  int maxHammingDistance_{100};
  int minTagArea_{0};
  int syncQueueSize_{100};
  double playbackRate_{1.0};
  double pixelNoise_{1.0};
  std::vector<cv::Mat> images_;
  std::vector<OdometryProcessor, Eigen::aligned_allocator<OdometryProcessor>>
    odomProcessors_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  rclcpp::Service<Trigger>::SharedPtr plotService_;
  rclcpp::Service<Trigger>::SharedPtr replayService_;
  rclcpp::Service<Trigger>::SharedPtr dumpService_;

  Profiler profiler_;
  std::list<uint64_t> times_;
  std::ofstream tagCornerFile_;
  string outBagName_;
  bool writeToBag_{false};
  bool publishInitialTransforms_{false};
  string optimizerMode_;
  string outDir_;
  string inBagFile_;
  std::shared_ptr<LiveExactSync> liveExactSync_;
  std::shared_ptr<LiveApproxSync> liveApproxSync_;

  std::unordered_map<int, std::vector<ReMap>> tagRemap_;
  std::vector<std::map<uint64_t, std::set<int>>> squash_;
  std::map<string, std::set<int>> camSquash_;
  std::vector<MeasurementsPtr> measurements_;
  PoseCacheMap poseCache_;
  uint64_t poseCacheTime_{0};
};
}  // namespace tagslam

#endif  // TAGSLAM__TAGSLAM_HPP_
