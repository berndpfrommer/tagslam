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

#include <tf2_ros/transform_broadcaster.h>

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
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <set>
#include <string>
#include <tagslam/camera.hpp>
#include <tagslam/graph.hpp>
#include <tagslam/graph_updater.hpp>
#include <tagslam/measurements/measurements.hpp>
#include <tagslam/odometry_processor.hpp>
#include <tagslam/pose_with_noise.hpp>
#include <tagslam/tag_factory.hpp>
#include <unordered_map>
#include <tagslam/profiler.hpp>

namespace tagslam
{
class Body;  // forward decl
class TagSLAM : public TagFactory, public rclcpp::Node
{
  using Apriltag = apriltag_msgs::msg::AprilTagDetection;
  using TagArray = apriltag_msgs::msg::AprilTagDetectionArray;
  using TagArrayPtr = TagArray::SharedPtr;
  using TagArrayConstPtr = TagArray::ConstSharedPtr;
  using Odometry = nav_msgs::msg::Odometry;
  using OdometryConstPtr = Odometry::ConstSharedPtr;
  using Image = sensor_msgs::msg::Image;
  using ImageConstPtr = Image::ConstSharedPtr;
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  using CompressedImageConstPtr = CompressedImage::ConstSharedPtr;
  using string = std::string;

  using ExactSync = flex_sync::ExactSync<TagArray, Image, Odometry>;
  using ApproxSync = flex_sync::ApproximateSync<TagArray, Image, Odometry>;
  using LiveExactSync = flex_sync::LiveSync<ExactSync>;
  using LiveApproxSync = flex_sync::LiveSync<ApproxSync>;

  using ExactCompressedSync =
    flex_sync::ExactSync<TagArray, CompressedImage, Odometry>;
  using ApproxCompressedSync =
    flex_sync::ApproximateSync<TagArray, CompressedImage, Odometry>;

  using LiveExactCompressedSync = flex_sync::LiveSync<ExactCompressedSync>;
  using LiveApproxCompressedSync = flex_sync::LiveSync<ApproxCompressedSync>;

#if 0
  using PoseCacheMap = std::map<
    std::string, PoseWithNoise, std::less<std::string>,
    Eigen::aligned_allocator<std::pair<std::string, PoseWithNoise>>>;
#endif
public:
  explicit TagSLAM(const rclcpp::NodeOptions & options);
  TagSLAM(const TagSLAM &) = delete;
  TagSLAM & operator=(const TagSLAM &) = delete;
  // inherited from TagFactory
  TagConstPtr findTag(int tagId) final;
#if 0
  // ------ own methods
  bool initialize();
  void run();
  void finalize(bool optimize = true);
  void subscribe();
  bool runOnline() const { return (inBagFile_.empty()); }

  template <typename SyncType, typename T1, typename T2, typename T3>
  void processBag(
    rosbag::View * view, const std::vector<std::vector<string>> & topics,
    std::function<void(
      const std::vector<typename T1::ConstPtr> &,
      const std::vector<typename T2::ConstPtr> &,
      const std::vector<typename T3::ConstPtr> &)> & cb)
  {
    SyncType sync(topics, cb, syncQueueSize_);
    for (const rosbag::MessageInstance & m : *view) {
      typename T1::ConstPtr t1 = m.instantiate<T1>();
      if (t1) {
        sync.process(m.getTopic(), t1);
      } else {
        typename T2::ConstPtr t2 = m.instantiate<T2>();
        if (t2) {
          sync.process(m.getTopic(), t2);
        } else {
          typename T3::ConstPtr t3 = m.instantiate<T3>();
          if (t3) {
            sync.process(m.getTopic(), t3);
          }
        }
      }
      if (frameNum_ > maxFrameNum_ || !ros::ok()) {
        break;
      }
    }
    if (sync.getNumberDropped() != 0) {
      ROS_WARN_STREAM("sync dropped messages: " << sync.getNumberDropped());
      sync.clearNumberDropped();
    }
  }

  void syncCallback(
    const std::vector<TagArrayConstPtr> & msgvec1,
    const std::vector<ImageConstPtr> & msgvec2,
    const std::vector<OdometryConstPtr> & msgvec3);
  void syncCallbackCompressed(
    const std::vector<TagArrayConstPtr> & msgvec1,
    const std::vector<CompressedImageConstPtr> & msgvec2,
    const std::vector<OdometryConstPtr> & msgvec3);
#endif
private:
  struct ReMap
  {
    ReMap(int i, const string & cam, uint64_t ts, uint64_t te)
    : remappedId(i), camera(cam), startTime(ts), endTime(te){};
    int remappedId;
    string camera;
    uint64_t startTime;
    uint64_t endTime;
  };
  typedef std::unordered_map<int, TagConstPtr> TagMap;
  // ---------- methods
  TagPtr addTag(int tagId, const std::shared_ptr<Body> & body) const;

#if 0
  void testForOldLaunchParameters();
  void readParams();
  void readBodies(XmlRpc::XmlRpcValue config);
  void readGlobalParameters(XmlRpc::XmlRpcValue config);
  void readCameras(XmlRpc::XmlRpcValue config);
  void readCameraPoses(XmlRpc::XmlRpcValue config);
  void readDistanceMeasurements();
  void readRemap(XmlRpc::XmlRpcValue config);
  void readSquash(XmlRpc::XmlRpcValue config);
  void parseTimeSquash(
    XmlRpc::XmlRpcValue sq, uint64_t t, const std::set<int> & tags);

  void playFromBag(const std::string & fname);
  void fakeOdom(uint64_t tCurr, std::vector<VertexDesc> * factors);

  void processOdom(
    uint64_t t, const std::vector<OdometryConstPtr> & odomMsg,
    std::vector<VertexDesc> * factors);
  std::vector<std::vector<std::string>> makeTopics() const;
  void testIfInBag(
    rosbag::Bag * bag,
    const std::vector<std::vector<std::string>> & topics) const;

  void setupOdom(const std::vector<OdometryConstPtr> & odomMsgs);
  void processTagsAndOdom(
    const std::vector<TagArrayConstPtr> & tagmsgs,
    const std::vector<OdometryConstPtr> & odommsgs);
  void publishTagAndBodyTransforms(uint64_t t, tf::tfMessage * tfMsg);
  void publishOriginalTagTransforms(uint64_t t, tf::tfMessage * tfMsg);
  void publishCameraTransforms(uint64_t t, tf::tfMessage * tfMsg);

  void publishTransforms(uint64_t t, bool orig = false);
  void publishBodyOdom(uint64_t t);
  void sleep(double dt) const;
  void processTags(
    uint64_t t, const std::vector<TagArrayConstPtr> & tagMsgs,
    std::vector<VertexDesc> * factors);
  std::vector<TagConstPtr> findTags(const std::vector<Apriltag> & ta);
  bool anyTagsVisible(const std::vector<TagArrayConstPtr> & tagmsgs);
  void publishAll(uint64_t t);
  bool plot(
    std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  bool replay(
    std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  bool dump(
    std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  void doDump(bool optimize);
  void writeCameraPoses(const string & fname);
  void writeFullCalibration(const string & fname) const;
  void writePoses(const string & fname);
  void writeTagDiagnostics(const string & fname) const;
  void writeTimeDiagnos]tics(const string & fname) const;
  void writeDistanceDiagnostics(const string & fname) const;
  void writeErrorMap(const string & fname) const;
  void writeTagCorners(
    uint64_t t, int camIdx, const TagConstPtr & tag,
    const geometry_msgs::Point * img_corners);

  void remapAndSquash(
    uint64_t t, std::vector<TagArrayConstPtr> * remapped,
    const std::vector<TagArrayConstPtr> & orig);
  void applyDistanceMeasurements();
  void doReplay(double rate);
  void copyPosesAndReset();
  PoseWithNoise getOptimizedPoseWithNoise(const string & name);
#endif
  // ------ variables --------

  TagMap tagMap_;
  std::shared_ptr<Body> defaultBody_;
  bool warnIgnoreTags_{false};
  GraphPtr graph_;

#if 0  
  GraphPtr initialGraph_;
  GraphUpdater graphUpdater_;
  CameraVec cameras_;
  BodyVec bodies_;
  BodyVec nonstaticBodies_;
    ros::Publisher clockPub_;
  ros::Publisher ackPub_;
  std::vector<ros::Publisher> odomPub_;
  std::vector<ros::Publisher> trajectoryPub_;
  std::vector<nav_msgs::Path> trajectory_;
  string fixedFrame_;
  bool writeDebugImages_;
  bool useApproximateSync_;
  bool hasCompressedImages_;
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
  tf::TransformBroadcaster tfBroadcaster_;
  ros::ServiceServer plotService_;
  ros::ServiceServer replayService_;
  ros::ServiceServer dumpService_;

  Profiler profiler_;
  std::list<uint64_t> times_;
  rosbag::Bag outBag_;
  std::ofstream tagCornerFile_;
  std::string outBagName_;
  bool writeToBag_{false};
  bool publishInitialTransforms_{false};
  std::string optimizerMode_;
  std::string outDir_;
  std::string inBagFile_;
  std::shared_ptr<LiveExactSync> liveExactSync_;
  std::shared_ptr<LiveApproximateSync> liveApproximateSync_;
  std::shared_ptr<LiveExactCompressedSync> liveExactCompressedSync_;
  std::shared_ptr<LiveApproximateCompressedSync> liveApproximateCompressedSync_;

  std::unordered_map<int, std::vector<ReMap>> tagRemap_;
  std::vector<std::map<uint64_t, std::set<int>>> squash_;
  std::map<std::string, std::set<int>> camSquash_;
  std::vector<MeasurementsPtr> measurements_;
  PoseCacheMap poseCache_;
  uint64_t poseCacheTime_{0};
#endif
};
}  // namespace tagslam

#endif  // TAGSLAM__TAGSLAM_HPP_