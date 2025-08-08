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

#ifdef USE_CV_BRIDGE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sstream>
#include <tagslam/body.hpp>
#include <tagslam/body_defaults.hpp>
#include <tagslam/factor/distance.hpp>
#include <tagslam/geometry.hpp>
#include <tagslam/graph_utils.hpp>
#include <tagslam/logging.hpp>
#include <tagslam/odometry_processor.hpp>
#include <tagslam/pose_with_noise.hpp>
#include <tagslam/tagslam.hpp>
#include <tagslam/yaml.hpp>
#include <tagslam/yaml_utils.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/transform_datatypes.hpp>
#include <unordered_set>

const int QSZ = 1000;

namespace tagslam
{
using Odometry = nav_msgs::msg::Odometry;
using OdometryConstPtr = Odometry::ConstSharedPtr;
using Image = sensor_msgs::msg::Image;
using ImageConstPtr = Image::ConstSharedPtr;
using CompressedImage = sensor_msgs::msg::CompressedImage;
using CompressedImageConstPtr = CompressedImage::ConstSharedPtr;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Point = apriltag_msgs::msg::Point;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using std::fixed;
using std::setprecision;
using std::setw;
using std::string;
#define FMT(X, Y) fixed << setw(X) << setprecision(Y)

static void poseEigenToMsg(
  const Eigen::Affine3d & e, geometry_msgs::msg::Pose & m)
{
  const auto & t = e.translation();
  m.position.x = t[0];
  m.position.y = t[1];
  m.position.z = t[2];
  const Eigen::Quaterniond q = (Eigen::Quaterniond)e.linear();
  m.orientation.x = q.x();
  m.orientation.y = q.y();
  m.orientation.z = q.z();
  m.orientation.w = q.w();
  if (m.orientation.w < 0) {
    m.orientation.x *= -1;
    m.orientation.y *= -1;
    m.orientation.z *= -1;
    m.orientation.w *= -1;
  }
}

static TransformStamped to_tftf(
  const rclcpp::Time & ts, const Transform & tf, const std::string & frame_id,
  const std::string & child_frame_id)
{
  TransformStamped tsm;
  tsm.header.stamp = ts;
  tsm.header.frame_id = frame_id;
  tsm.child_frame_id = child_frame_id;
  const auto q = Eigen::Quaterniond(tf.rotation());
  tsm.transform.rotation.x = q.x();
  tsm.transform.rotation.y = q.y();
  tsm.transform.rotation.z = q.z();
  tsm.transform.rotation.w = q.w();
  tsm.transform.translation.x = tf.translation().x();
  tsm.transform.translation.y = tf.translation().y();
  tsm.transform.translation.z = tf.translation().z();
  return (tsm);
}

static void write_time(std::ostream & o, uint64_t t)
{
  o << (t / 1000000000ULL) << "." << std::setfill('0') << setw(9)
    << (t % 1000000000ULL) << " " << std::setfill(' ');
}

static void write_vec(std::ostream & o, const Eigen::Vector3d & v)
{
  for (int i = 0; i < 3; i++) {
    o << " " << FMT(7, 3) << v(i);
  }
}

static double find_size_of_tag(const Point * imgCorn)
{
  Eigen::Matrix<double, 4, 2> x;
  x << imgCorn[0].x, imgCorn[0].y, imgCorn[1].x, imgCorn[1].y, imgCorn[2].x,
    imgCorn[2].y, imgCorn[3].x, imgCorn[3].y;
  // shoelace formula
  const double A = fabs(
    x(0, 0) * x(1, 1) + x(1, 0) * x(2, 1) + x(2, 0) * x(3, 1) +
    x(3, 0) * x(0, 1) - x(1, 0) * x(0, 1) - x(2, 0) * x(1, 1) -
    x(3, 0) * x(2, 1) - x(0, 0) * x(3, 1));
  return (A);
}

const std::map<std::string, OptimizerMode> optModeMap = {
  {"full", SLOW}, {"slow", SLOW}, {"fast", FAST}};

TagSLAM::TagSLAM(const rclcpp::NodeOptions & options) : Node("tagslam", options)
{
  node_ = this;
  tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  initialGraph_.reset(new Graph());
  // Alias the graph to the initial graph. That way during startup,
  // all updates that are done on graph_, are also done on the initial
  // graph
  graph_ = initialGraph_;
  graph_->setVerbosity("SILENT");
  initialize();
  subscribe();
}

TagConstPtr TagSLAM::findTag(int tagId)
{
  TagMap::iterator it = tagMap_.find(tagId);
  TagPtr p;
  if (it == tagMap_.end()) {
    if (!defaultBody_) {
      if (warnIgnoreTags_) {
        LOG_WARN("no default body, ignoring tag: " << tagId);
      }
      return (p);
    } else {
      p = addTag(tagId, defaultBody_);
      if (!p) {
        return (p);
      }
      auto iit = tagMap_.insert(TagMap::value_type(tagId, p));
      it = iit.first;
    }
  }
  return (it->second);
}

TagPtr TagSLAM::addTag(int tagId, const std::shared_ptr<Body> & body) const
{
  TagPtr p;
  if (body->ignoreTag(tagId)) {
    return (p);
  }
  p = Tag::make(
    tagId, 6 /*num bits = tag family */, body->getDefaultTagSize(),
    PoseWithNoise() /* invalid pose */, body);
  body->addTag(p);
  LOG_INFO("new tag " << tagId << " attached to " << body->getName());
  graph_utils::add_tag(graph_.get(), *p);
  return (p);
}

void TagSLAM::readParams()
{
  outBagName_ = declare_parameter("outbag", "out.bag");
  playbackRate_ = declare_parameter("playback_rate", 5.0);
  outDir_ = declare_parameter("output_directory", ".");
  fixedFrame_ = declare_parameter<string>("fixed_frame_id", "map");
  maxFrameNum_ = declare_parameter<int>("max_number_of_frames", 0);
  publishAck_ = declare_parameter<bool>("publish_ack", false);
}

static YAML::Node readConfig(
  rclcpp::Node * node, const std::string & param_name,
  bool can_be_empty = false)
{
  auto logger = node->get_logger();
  const std::string conf_file = node->declare_parameter<string>(param_name, "");
  if (conf_file.empty()) {
    if (!can_be_empty) {
      RCLCPP_ERROR_STREAM(
        logger, param_name << " parameter must be set to valid filename!");
      throw(std::runtime_error("param " + param_name + " not set"));
    } else {
      return (YAML::Node());
    }
  }
  YAML::Node config;
  try {
    config = YAML::LoadFile(conf_file);
  } catch (const YAML::ParserException & e) {
    RCLCPP_ERROR_STREAM(
      logger, "error parsing file: " << conf_file << ": " << e.what());
    throw(std::runtime_error("error parsing file: " + conf_file));
  } catch (const YAML::BadFile & e) {
    RCLCPP_ERROR_STREAM(
      logger, "bad or non-existent file: " << conf_file << ": " << e.what());
    throw(std::runtime_error("bad file: " + conf_file));
  }
  if (config.IsNull()) {
    RCLCPP_ERROR_STREAM(logger, "cannot open config file: " << conf_file);
    throw(std::runtime_error("cannot open config file " + conf_file));
  }
  return (config);
}

bool TagSLAM::initialize()
{
  readParams();
  auto config = readConfig(node_, "tagslam_config");
  graphUpdater_.parse(config);
  const auto ommi = optModeMap.find(graphUpdater_.getOptimizerMode());
  if (ommi == optModeMap.end()) {
    BOMB_OUT("invalid optimizer mode: " << optimizerMode_);
  } else {
    graph_->getOptimizer()->setMode(ommi->second);
  }
  LOG_INFO("optimizer mode: " << graphUpdater_.getOptimizerMode());

  readBodies(config);
  readGlobalParameters(config);
  readCameras(readConfig(node_, "cameras"));
  readSquash(config);
  readRemap(config);
  readCameraPoses(readConfig(node_, "camera_poses", true));
  measurements_ = measurements::read_all(config, this);
  // apply measurements
  for (auto & m : measurements_) {
    m->addToGraph(graph_);
    m->tryAddToOptimizer(graph_);
  }
  replayService_ = node_->create_service<Trigger>(
    "replay",
    std::bind(
      &TagSLAM::replay, this, std::placeholders::_1, std::placeholders::_2));
  dumpService_ = node_->create_service<Trigger>(
    "dump",
    std::bind(
      &TagSLAM::dump, this, std::placeholders::_1, std::placeholders::_2));
  plotService_ = node_->create_service<Trigger>(
    "plot",
    std::bind(
      &TagSLAM::plot, this, std::placeholders::_1, std::placeholders::_2));
  if (publishAck_) {
    ackPub_ = node_->create_publisher<Header>("acknowledge", 10);
  }
  // optimize the initial setup if necessary
  graph_->optimize(0);
  // open output files
  tagCornerFile_.open("tag_corners.txt");
  // make a deep copy of the initial graph now
  graph_.reset(initialGraph_->clone());
  return (true);
}

void TagSLAM::subscribe()
{
  std::vector<std::vector<std::string>> topics = makeTopics();
  if (useApproximateSync_) {
    liveApproxSync_.reset(new LiveApproxSync(
      node_, topics,
      std::bind(
        &TagSLAM::syncCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      syncQueueSize_));
  } else {
    liveExactSync_.reset(new LiveExactSync(
      node_, topics,
      std::bind(
        &TagSLAM::syncCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      syncQueueSize_));
  }
}

static void makeTopic(
  rosbag2_cpp::Writer * writer, const std::string & topic,
  const std::string & type)
{
  struct rosbag2_storage::TopicMetadata md;
  md.name = topic;
  md.type = type;
  md.serialization_format = rmw_get_serialization_format();
  // md.offered_qos_profiles = "";
  writer->create_topic(md);
}

void TagSLAM::openOutputBag(const string & bag_name)
{
  outputBag_ = std::make_unique<rosbag2_cpp::Writer>();
  if (bag_name.find('*') != std::string::npos) {
    BOMB_OUT("cannot have wildcards in bag name!");
  }
  using Path = std::filesystem::path;
  if (std::filesystem::exists(Path(bag_name) / Path("metadata.yaml"))) {
    LOG_INFO("removing existing bag: " << bag_name);
    std::filesystem::remove_all(bag_name);
  }

  outputBag_->open(bag_name);
  makeTopic(outputBag_.get(), "/tf2", "tf2_msgs/msg/TFMessage");
  struct rosbag2_storage::TopicMetadata md;
  for (const auto & body : bodies_) {
    makeTopic(
      outputBag_.get(), "/tagslam/odom/body_" + body->getName(),
      "nav_msgs/msg/Odometry");
  }
}

void TagSLAM::doDump(bool optimize)
{
  graphUpdater_.printPerformance();
  // do final optimization
  if (optimize) {
    profiler_.reset("finalOptimization");
    const double error = graph_->optimizeFull(true);  // true === force
    profiler_.record("finalOptimization");
    LOG_INFO("final error: " << error);
  }
  graph_->printUnoptimized();
  for (auto & m : measurements_) {
    m->printUnused(graph_);
  }
  // add a little delta to avoid duplicate time stamp warnings
  publishTransforms(
    times_.empty() ? 0ULL : (*(times_.rbegin()) + 1000ULL), true);
  tagCornerFile_.flush();
  openOutputBag(outBagName_);
  writeToBag_ = true;
  doReplay(0);  //  0 = playback at full speed
  writeToBag_ = false;
  outputBag_->close();
  outputBag_.reset();

  writeCameraPoses(outDir_ + "/camera_poses.yaml");
  writeFullCalibration(outDir_ + "/calibration.yaml");
  profiler_.reset("writePoses");
  writePoses(outDir_ + "/poses.yaml");
  profiler_.record("writePoses");
  profiler_.reset("writeErrorMap");
  writeErrorMap(outDir_ + "/error_map.txt");
  profiler_.record("writeErrorMap");
  profiler_.reset("writeTagDiagnostics");
  writeTagDiagnostics(outDir_ + "/tag_diagnostics.txt");
  profiler_.record("writeTagDiagnostics");
  profiler_.reset("writeTimeDiagnostics");
  writeTimeDiagnostics(outDir_ + "/time_diagnostics.txt");
  profiler_.record("writeTimeDiagnostics");
  profiler_.reset("writeMeasurementDiagnostics");
  for (auto & m : measurements_) {
    m->writeDiagnostics(graph_);
  }
  profiler_.record("writeMeasurementDiagnostics");
  std::cout << profiler_ << std::endl;
  std::cout.flush();
}

void TagSLAM::finalize(bool optimize)
{
  doDump(optimize);
  tagCornerFile_.close();
  for (auto const & op : odomProcessors_) {
    op.finalize();
  }
}

void TagSLAM::readBodies(const YAML::Node & config)
{
  // read body defaults first in case
  // bodies do not provide all parameters
  BodyDefaults::parse(config);  // keeps global state!
  // now read bodies
  bodies_ = Body::parse_bodies(config);
  for (const auto & body : bodies_) {
    graph_utils::add_body(graph_.get(), *body);
    // add associated tags as vertices
    for (const auto & tag : body->getTags()) {
      tagMap_.insert(TagMap::value_type(tag->getId(), tag));
    }
    if (!body->isStatic()) {
      if (
        body->getFakeOdomTranslationNoise() > 0 &&
        body->getFakeOdomRotationNoise() > 0) {
        useFakeOdom_ = true;
        LOG_INFO("using fake odom for " << body->getName());
      }
      nonstaticBodies_.push_back(body);
      odomPub_.push_back(
        node_->create_publisher<Odometry>("odom/body_" + body->getName(), QSZ));
      pathPub_.push_back(
        node_->create_publisher<Path>("path/body_" + body->getName(), QSZ));
      trajectory_.push_back(Path());
    }
  }
}

void TagSLAM::readGlobalParameters(const YAML::Node & config)
{
  const string defbody = yaml::parse<std::string>(config, "default_body", "");
  warnIgnoreTags_ = yaml::parse<bool>(config, "warn_ignore_tags", false);
  useApproximateSync_ = yaml::parse<bool>(
    config["tagslam_parameters"], "use_approximate_sync", false);
  syncQueueSize_ =
    yaml::parse<int>(config["tagslam_parameters"], "sync_queue_size", 100);
  minTagArea_ =
    yaml::parse<int>(config["tagslam_parameters"], "minimum_tag_area", 0);
  if (defbody.empty()) {
    LOG_WARN("no default body specified!");
  } else {
    for (auto & body : bodies_) {
      if (body->getName() == defbody) {
        defaultBody_ = body;
        if (defaultBody_->getDefaultTagSize() <= 0) {
          BOMB_OUT("body " << defbody << " must have default tag size!");
        }
        break;
      }
    }
    if (!defaultBody_) {
      BOMB_OUT("cannot find default body: " << defbody);
    }
  }
  amnesia_ = yaml::parse<bool>(config, "amnesia", false);
  if (amnesia_) {
    LOG_INFO("using amnesia!");
  }
  maxHammingDistance_ = yaml::parse<int>(config, "max_hamming_distance", 100);
}

void TagSLAM::readCameraPoses(const YAML::Node & config)
{
  std::map<BodyConstPtr, int> numKnownCamPoses;
  for (auto & cam : cameras_) {
    if (numKnownCamPoses.count(cam->getRig()) == 0) {
      numKnownCamPoses.emplace(cam->getRig(), 0);
    }
    PoseWithNoise pwn;  // defaults to invalid pose
    if (config[cam->getName()]) {
      pwn = yaml::parse<PoseWithNoise>(
        config[cam->getName()], "pose", PoseWithNoise());
      if (pwn.isValid()) {
        numKnownCamPoses[cam->getRig()]++;
        LOG_INFO("camera " << cam->getName() << " has known pose!");
      }
    }
    graph_utils::add_pose_maybe_with_prior(
      graph_.get(), 0, Graph::cam_name(cam->getName()), pwn, true);
  }
  for (const auto & ncp : numKnownCamPoses) {
    if (ncp.first->isStatic()) {
      if (ncp.second == 0 && !ncp.first->getPoseWithNoise().isValid()) {
        BOMB_OUT(ncp.first->getName() << " has no pose nor cam pose!");
      }
    } else {
      if (ncp.second == 0) {
        BOMB_OUT(ncp.first->getName() << " must have cam with known pose!");
      }
    }
  }
}

void TagSLAM::readCameras(const YAML::Node & config)
{
  if (!config.IsDefined()) {
    BOMB_OUT("no camera configurations found!");
  }
  cameras_ = Camera::parse_cameras(config);
  LOG_INFO("found " << cameras_.size() << " cameras");
  for (auto & cam : cameras_) {
    for (const auto & body : bodies_) {
      if (body->getName() == cam->getRigName()) {
        cam->setRig(body);
      }
    }
    if (!cam->getRig()) {
      BOMB_OUT("rig body not found: " << cam->getRigName());
    }
  }
}

std::vector<std::string> TagSLAM::getTagTopics() const
{
  std::vector<std::string> tagTopics;
  for (const auto & c : cameras_) {
    tagTopics.push_back(c->getTagTopic());
  }
  return (tagTopics);
}

std::vector<std::pair<std::string, std::string>> TagSLAM::getImageTopics() const
{
  std::vector<std::pair<std::string, std::string>> imageTopics;
  for (const auto & c : cameras_) {
    imageTopics.push_back({c->getImageTopic(), c->getImageTransport()});
  }
  return (imageTopics);
}

std::vector<std::string> TagSLAM::getOdomTopics() const
{
  std::vector<std::string> odomTopics;
  for (const auto & body : bodies_) {
    if (!body->getOdomTopic().empty()) {
      odomTopics.push_back(body->getOdomTopic());
    }
  }
  return (odomTopics);
}

std::vector<std::string> TagSLAM::getPublishedTopics() const
{
  std::vector<std::string> pubTopics;
  for (const auto & body : bodies_) {
    if (!body->getOdomTopic().empty()) {
      pubTopics.push_back("/tagslam/odom/body_" + body->getName());
    }
  }
  return (pubTopics);
}

std::vector<std::vector<std::string>> TagSLAM::makeTopics() const
{
  std::vector<std::vector<string>> topics;
  topics.push_back(getTagTopics());
  topics.push_back(getOdomTopics());
  return (topics);
}

void TagSLAM::plot(
  const std::shared_ptr<Trigger::Request>,
  const std::shared_ptr<Trigger::Response> res)
{
  LOG_INFO("plotting!");
  graph_utils::plot("graph.dot", graph_.get());
  res->message = "dump complete!";
  res->success = true;
  LOG_INFO("finished dumping.");
}

void TagSLAM::replay(
  const std::shared_ptr<Trigger::Request>,
  const std::shared_ptr<Trigger::Response> res)
{
  LOG_INFO("replaying!");
  doReplay(playbackRate_);
  res->message = "replayed " + std::to_string(times_.size());
  res->success = true;
  LOG_INFO("finished replaying " << times_.size() << " frames");
}

void TagSLAM::dump(
  const std::shared_ptr<Trigger::Request>,
  const std::shared_ptr<Trigger::Response> res)
{
  LOG_INFO("dumping!");
  doDump(true);
  res->message = "dump complete!";
  res->success = true;
  LOG_INFO("finished dumping.");
}

void TagSLAM::doReplay(double rate)
{
  rclcpp::WallRate wall_rate(std::max(rate, 1e-3));
  for (const auto & t : times_) {
    publishAll(t);
    if (rate > 0) {
      wall_rate.sleep();
    }
  }
  if (!times_.empty()) {
    publishTransforms(*times_.rbegin(), true);
  }
}

void TagSLAM::publishCameraTransforms(const uint64_t t, TFMessage * tfMsg)
{
  for (const auto & cam : cameras_) {
    Transform camTF;
    if (graph_utils::get_optimized_pose(*graph_, *cam, &camTF)) {
      const string & rigFrameId = cam->getRig()->getFrameId();
      const auto ctf =
        to_tftf(rosTime(t), camTF, rigFrameId, cam->getFrameId());
      if (!writeToBag_) {
        tfBroadcaster_->sendTransform(ctf);
      }
      tfMsg->transforms.push_back(ctf);
    }
  }
}

void TagSLAM::publishTagAndBodyTransforms(uint64_t t, TFMessage * tfMsg)
{
  TransformStamped tfm;
  for (const auto & body : bodies_) {
    Transform bodyTF;
    const string & bodyFrameId = body->getFrameId();
    const uint64_t ts = body->isStatic() ? 0 : t;
    if (graph_utils::get_optimized_pose(*graph_, ts, *body, &bodyTF)) {
      const auto btf = to_tftf(rosTime(t), bodyTF, fixedFrame_, bodyFrameId);
      tfMsg->transforms.push_back(btf);
      if (!writeToBag_) {
        tfBroadcaster_->sendTransform(btf);
      }
      for (const auto & tag : body->getTags()) {
        Transform tagTF;
        if (graph_utils::get_optimized_pose(*graph_, *tag, &tagTF)) {
          const std::string frameId = "tag_" + std::to_string(tag->getId());
          const auto ttf = to_tftf(rosTime(t), tagTF, bodyFrameId, frameId);
          if (!writeToBag_) {
            tfBroadcaster_->sendTransform(ttf);
          }
          tfMsg->transforms.push_back(ttf);
        }
      }
    }
  }
}

void TagSLAM::publishOriginalTagTransforms(const uint64_t t, TFMessage * tfMsg)
{
  TransformStamped tfm;
  for (const auto & body : bodies_) {
    if (!body->getPoseWithNoise().isValid()) {
      continue;
    }
    const string & bodyFrameId = body->getFrameId();
    for (const auto & tag : body->getTags()) {
      if (tag->getPoseWithNoise().isValid()) {
        const Transform tagTF = tag->getPoseWithNoise().getPose();
        const std::string frameId = "o_tag_" + std::to_string(tag->getId());
        const auto ttf = to_tftf(rosTime(t), tagTF, bodyFrameId, frameId);
        if (!writeToBag_) {
          tfBroadcaster_->sendTransform(ttf);
        }
        tfMsg->transforms.push_back(ttf);
      }
    }
  }
}

void TagSLAM::publishTransforms(const uint64_t t, bool orig)
{
  TFMessage tfMsg;
  publishTagAndBodyTransforms(t, &tfMsg);
  if (orig) {
    publishOriginalTagTransforms(t, &tfMsg);
  }
  publishCameraTransforms(t, &tfMsg);
  if (t != 0 && writeToBag_) {
    outputBag_->write<TFMessage>(tfMsg, "/tf2", rosTime(t));
  }
}

void TagSLAM::syncCallback(
  const std::vector<TagArrayConstPtr> & msgvec1,
  const std::vector<OdometryConstPtr> & msgvec3)
{
  profiler_.reset("processTagsAndOdom");
  processTagsAndOdom(msgvec1, msgvec3);
  profiler_.record("processTagsAndOdom");
}

static Odometry make_odom(
  const rclcpp::Time & t, const std::string & fixed_frame,
  const std::string & child_frame, const PoseWithNoise & pwn)
{
  Odometry odom;
  odom.header.stamp = t;
  odom.header.frame_id = fixed_frame;
  odom.child_frame_id = child_frame;
  poseEigenToMsg(pwn.getPose(), odom.pose.pose);
  memcpy(
    &odom.pose.covariance[0], &pwn.getNoise().getCovarianceMatrix()(0),
    sizeof(odom.pose.covariance));
  return (odom);
}

void TagSLAM::publishBodyOdom(const uint64_t t)
{
  for (size_t body_idx = 0; body_idx < nonstaticBodies_.size(); body_idx++) {
    const auto body = nonstaticBodies_[body_idx];
    PoseWithNoise pwn;
    if (body->publishCovariance()) {
      pwn = graph_utils::get_optimized_pose_with_noise(
        *graph_, t, Graph::body_name(body->getName()));
    } else {
      Transform pose;
      const bool isValid =
        graph_utils::get_optimized_pose(*graph_, t, *body, &pose);
      pwn = PoseWithNoise(pose, PoseNoise(), isValid);
    }
    if (pwn.isValid()) {
      auto msg =
        make_odom(rosTime(t), fixedFrame_, body->getOdomFrameId(), pwn);
      if (writeToBag_) {
        outputBag_->write<Odometry>(
          msg, "/tagslam/odom/body_" + body->getName(), rosTime(t));
      } else {
        odomPub_[body_idx]->publish(msg);
        PoseStamped pose_msg;
        pose_msg.header.stamp = rosTime(t);
        pose_msg.header.frame_id = body->getOdomFrameId();
        pose_msg.pose = msg.pose.pose;

        trajectory_[body_idx].header.stamp = rosTime(t);
        trajectory_[body_idx].header.frame_id = fixedFrame_;
        trajectory_[body_idx].poses.push_back(pose_msg);
        pathPub_[body_idx]->publish(trajectory_[body_idx]);
      }
    }
  }
}

bool TagSLAM::anyTagsVisible(const std::vector<TagArrayConstPtr> & tagmsgs)
{
  for (const auto & msg : tagmsgs) {
    if (!findTags(msg->detections).empty()) {
      return (true);
    }
  }
  return (false);
}

void TagSLAM::copyPosesAndReset()
{
  //
  // For all dynamic bodies, copy the previous pose
  // and bolt it down with a pose prior. In combination
  // with fake odometry, this helps initialization.
  //
  const auto t = times_.back();
  Graph * g = initialGraph_->clone();
  for (const auto & body : bodies_) {
    if (body->isStatic() && body->getPoseWithNoise().isValid()) {
      continue;
    }
    Transform pose;
    if (graph_utils::get_optimized_pose(*graph_, t, *body, &pose)) {
      const std::string name = Graph::body_name(body->getName());
      // add pose to graph and optimizer
      const VertexDesc v = g->addPose(t, name, false);  // false = isCamPose
      const PoseValuePtr pp = std::dynamic_pointer_cast<value::Pose>((*g)[v]);
      pp->addToOptimizer(pose, g);
      // add pose prior to graph and optimizer
      const double ns = 0.001;
      PoseWithNoise pwn(pose, PoseNoise::make(ns, ns), true);
      AbsolutePosePriorFactorPtr app(
        new factor::AbsolutePosePrior(t, pwn, pp->getName()));
      app->addToGraph(app, g);
      app->addToOptimizer(g);
    }
  }
  times_.clear();
  times_.push_back(t);  // such that fake odom works!
  graph_.reset(g);      // now use the new graph
}

void TagSLAM::processTagsAndOdom(
  const std::vector<TagArrayConstPtr> & origtagmsgs,
  const std::vector<OdometryConstPtr> & odommsgs)
{
  profiler_.reset("processOdom");
  if (amnesia_ && !times_.empty()) {
    copyPosesAndReset();
  }
  if (origtagmsgs.empty() && odommsgs.empty()) {
    LOG_WARN("got called with neither tags nor odom!");
    return;
  }
  // the first odom or tag message determines the time stamp
  const auto & header =
    origtagmsgs.empty() ? odommsgs[0]->header : origtagmsgs[0]->header;
  const uint64_t t = rclcpp::Time(header.stamp).nanoseconds();
  if (publishInitialTransforms_) {
    publishTransforms(t, false /*dont pub orig tf */);
    publishInitialTransforms_ = false;
  }
  std::vector<TagArrayConstPtr> tagmsgs;
  remapAndSquash(t, &tagmsgs, origtagmsgs);
  if (tagmsgs.empty() && odommsgs.empty()) {
    LOG_WARN("squashed hard: have neither tags nor odom!");
    return;
  }
  if (!times_.empty() && t <= times_.back()) {
    LOG_WARN("received old time stamp: " << t);
    return;
  }
  const bool hasOdom = !odommsgs.empty() || useFakeOdom_;
  if (anyTagsVisible(tagmsgs) || hasOdom) {
    // if we have any new valid observations,
    // add unknown poses for all non-static bodies
    for (const auto & body : nonstaticBodies_) {
      graph_->addPose(t, Graph::body_name(body->getName()), false);
    }
  }
  std::vector<VertexDesc> factors;
  if (odommsgs.size() != 0) {
    processOdom(t, odommsgs, &factors);
  } else if (useFakeOdom_) {
    fakeOdom(t, &factors);
  }
  profiler_.record("processOdom");
  profiler_.reset("processTags");
  processTags(t, tagmsgs, &factors);
  profiler_.record("processTags");
  profiler_.reset("processNewFactors");
  try {
    graphUpdater_.processNewFactors(graph_.get(), t, factors);
  } catch (const OptimizerException & e) {
    LOG_WARN("optimizer crapped out!");
    LOG_WARN(e.what());
    finalize(false);
    throw(e);
  }
  profiler_.record("processNewFactors");
  profiler_.reset("publish");
  times_.push_back(t);
  publishAll(t);
  frameNum_++;
  if (publishAck_) {
    ackPub_->publish(header);
  }
  profiler_.record("publish");
  if (maxFrameNum_ && frameNum_ >= maxFrameNum_) {
    if (publishAck_) {
      auto h = header;
      h.frame_id = "FINISHED!";
      ackPub_->publish(h);
    }
    LOG_INFO("reached max number of frames, finished!");
    finalize(true);
  }
}

void TagSLAM::publishAll(const uint64_t t)
{
  publishBodyOdom(t);
  publishTransforms(t, false);
}

void TagSLAM::fakeOdom(uint64_t tCurr, std::vector<VertexDesc> * factors)
{
  if (!times_.empty()) {
    const auto & tPrev = times_.back();
    for (const auto & body : bodies_) {
      if (!body->isStatic()) {
        const PoseNoise pn = PoseNoise::make(
          body->getFakeOdomRotationNoise(),
          body->getFakeOdomTranslationNoise());
        const PoseWithNoise pwn(Transform::Identity(), pn, true);
        factors->push_back(OdometryProcessor::add_body_pose_delta(
          graph_.get(), tPrev, tCurr, body, pwn));
      }
    }
  }
}

void TagSLAM::setupOdom(const std::vector<OdometryConstPtr> & odomMsgs)
{
  std::set<BodyConstPtr> bodySet;
  for (size_t odomIdx = 0; odomIdx < odomMsgs.size(); odomIdx++) {
    const auto & frameId = odomMsgs[odomIdx]->child_frame_id;
    BodyConstPtr bpt;
    for (const auto & body : bodies_) {
      if (body->getOdomFrameId() == frameId) {
        bpt = body;
      }
    }
    if (!bpt) {
      BOMB_OUT("no body found for odom frame id: " << frameId);
    }
    if (bodySet.count(bpt) != 0) {
      LOG_WARN("multiple bodies with frame id: " << frameId);
      LOG_WARN("This will screw up the odom!");
    }
    LOG_INFO("have odom from " << bpt->getName() << " " << frameId);
    odomProcessors_.push_back(OdometryProcessor(node_, bpt));
  }
}

void TagSLAM::processOdom(
  uint64_t t, const std::vector<OdometryConstPtr> & odomMsgs,
  std::vector<VertexDesc> * factors)
{
  if (odomProcessors_.empty()) {
    setupOdom(odomMsgs);
  }
  // from odom child frame id, deduce bodies
  for (size_t odomIdx = 0; odomIdx < odomMsgs.size(); odomIdx++) {
    const auto & msg = odomMsgs[odomIdx];
    odomProcessors_[odomIdx].process(t, graph_.get(), msg, factors);
    // graph_.test();
  }
}

void TagSLAM::writeCameraPoses(const string & fname)
{
  std::ofstream f(fname);
  for (const auto & cam : cameras_) {
    f << cam->getName() << ":" << std::endl;
    try {
      PoseWithNoise pwn =
        getOptimizedPoseWithNoise(Graph::cam_name(cam->getName()));
      if (pwn.isValid()) {
        f << "  pose:" << std::endl;
        yaml_utils::write_pose_with_covariance(
          f, "    ", pwn.getPose(), pwn.getNoise());
      }
    } catch (const OptimizerException & e) {
      LOG_WARN("no optimized pose for: " << cam->getName());
    }
  }
}

PoseWithNoise TagSLAM::getOptimizedPoseWithNoise(const string & name)
{
  if (times_.back() > poseCacheTime_) {
    // if new data has come in, invalidate the pose cache
    poseCache_.clear();
    poseCacheTime_ = times_.back();
  }
  const auto it = poseCache_.find(name);
  if (it == poseCache_.end()) {
    const PoseWithNoise pwn =
      graph_utils::get_optimized_pose_with_noise(*graph_, 0, name);
    if (pwn.isValid()) {
      poseCache_.insert(PoseCacheMap::value_type(name, pwn));
    }
    return (pwn);
  }
  return (it->second);
}

void TagSLAM::writeFullCalibration(const string & fname) const
{
  std::ofstream f(fname);
  for (const auto & cam : cameras_) {
    Transform tf;
    if (graph_utils::get_optimized_pose(
          *graph_, 0, Graph::cam_name(cam->getName()), &tf)) {
      f << cam->getName() << ":" << std::endl;
      f << "  T_cam_body:" << std::endl;
      yaml_utils::write_matrix(f, "  ", tf.inverse());
      cam->getIntrinsics().writeYaml(f, "  ");
    } else {
      LOG_WARN("no pose found for camera: " << cam->getName());
    }
  }
}

void TagSLAM::writePoses(const string & fname)
{
  std::ofstream f(fname);
  f << "bodies:" << std::endl;
  const std::string idn = "       ";
  for (const auto & body : bodies_) {
    Transform bodyTF;
    body->write(f, " ");
    if (body->isStatic()) {
      try {
        PoseWithNoise pwn =
          getOptimizedPoseWithNoise(Graph::body_name(body->getName()));
        if (pwn.isValid()) {
          f << "     pose:" << std::endl;
          yaml_utils::write_pose(
            f, "       ", pwn.getPose(), pwn.getNoise(), true);
        }
      } catch (const OptimizerException & e) {
        LOG_WARN("cannot find pose for " << body->getName());
      }
    }
    if (body->printTags()) {
      f << "     tags:" << std::endl;
      for (const auto & tag : body->getTags()) {
        try {
          const auto pwn = graph_utils::get_optimized_pose_with_noise(
            *graph_, 0, Graph::tag_name(tag->getId()));
          if (pwn.isValid()) {
            f << idn << "- id: " << tag->getId() << std::endl;
            f << idn << "  size: " << tag->getSize() << std::endl;
            f << idn << "  pose:" << std::endl;
            // const auto &pwn = tag->getPoseWithNoise(); // orig noise
            yaml_utils::write_pose(
              f, idn + "    ", pwn.getPose(), pwn.getNoise(), true);
          }
        } catch (const OptimizerException & e) {
          LOG_WARN("cannot find pose for tag " << tag->getId());
        }
      }
    }
  }
}

void TagSLAM::writeTimeDiagnostics(const string & fname) const
{
  std::ofstream f(fname);
  const Graph::TimeToErrorMap m = graph_->getTimeToErrorMap();
  for (const auto & te : m) {
    write_time(f, te.first);
    double err(0);
    for (const auto & fe : te.second) {
      err += fe.second;
    }
    f << err;
    for (const auto & fe : te.second) {
      f << " " << (*fe.first) << ":err=" << fe.second;
    }
    f << std::endl;
  }
}

void TagSLAM::writeErrorMap(const string & fname) const
{
  std::ofstream f(fname);
  const auto errMap = graph_->getErrorMap();
  for (const auto & v : errMap) {
    const auto & vp = graph_->getVertex(v.second);
    f << FMT(8, 3) << v.first << " ";
    write_time(f, vp->getTime());
    f << " " << *vp << std::endl;
  }
}

void TagSLAM::writeTagDiagnostics(const string & fname) const
{
  std::ofstream f(fname);
  const std::string idn = "       ";
  for (const auto & body : bodies_) {
    for (const auto & tag : body->getTags()) {
      Transform tagTF;
      if (graph_utils::get_optimized_pose(*graph_, *tag, &tagTF)) {
        const auto & pwn = tag->getPoseWithNoise();
        if (pwn.isValid()) {
          const Transform poseDiff = pwn.getPose().inverse() * tagTF;
          const auto x = poseDiff.translation();
          Eigen::AngleAxisd aa;
          aa.fromRotationMatrix(poseDiff.rotation());
          f << setw(3) << tag->getId() << " " << FMT(6, 3) << x.norm();
          write_vec(f, x);
          const auto w = aa.angle() * aa.axis();
          f << "   ang: " << aa.angle() * 180 / M_PI;
          write_vec(f, w);
          f << std::endl;
        }
      }
    }
  }
}

std::vector<TagConstPtr> TagSLAM::findTags(const std::vector<Apriltag> & ta)
{
  std::vector<TagConstPtr> tpv;
  for (const auto & tag : ta) {
    TagConstPtr tagPtr = findTag(tag.id);
    if (tagPtr) {
      tpv.push_back(tagPtr);
    }
  }
  return (tpv);
}

void TagSLAM::processTags(
  uint64_t t, const std::vector<TagArrayConstPtr> & tagMsgs,
  std::vector<VertexDesc> * factors)
{
  if (tagMsgs.size() != cameras_.size()) {
    BOMB_OUT(
      "tag msgs size mismatch: " << tagMsgs.size() << " " << cameras_.size());
  }
  typedef std::multimap<double, VertexDesc> MMap;
  MMap sortedFactors;

  for (size_t i = 0; i < cameras_.size(); i++) {
    const auto & cam = cameras_[i];
    const auto tags = findTags(tagMsgs[i]->detections);
    if (!tags.empty()) {
      // insert time-dependent camera pose
      graph_->addPose(t, Graph::cam_name(cam->getName()), true /*camPose*/);
      // and tie it to the time-independent camera pose
      // with a relative prior
      const PoseWithNoise pn(Transform::Identity(), cam->getWiggle(), true);
      string name = Graph::cam_name(cam->getName());
      RelativePosePriorFactorPtr fac(
        new factor::RelativePosePrior(t, 0, pn, name));
      VertexDesc v = fac->addToGraph(fac, graph_.get());
      sortedFactors.insert(MMap::value_type(1e10, v));
    }
    std::unordered_set<int> tagsFound;
    for (const auto & tag : tagMsgs[i]->detections) {
      TagConstPtr tagPtr = findTag(tag.id);
      if (!tagPtr) {
        continue;
      }
      if (tagsFound.count(tag.id) == 0) {
        const auto * corners = &(tag.corners[0]);
        TagProjectionFactorPtr fp(new factor::TagProjection(
          t, cam, tagPtr, corners, graphUpdater_.getPixelNoise(),
          cam->getName() + "-" + Graph::tag_name(tagPtr->getId())));
        auto fac = fp->addToGraph(fp, graph_.get());
        double sz = find_size_of_tag(corners);
        if (sz < minTagArea_) {
          LOG_WARN(
            "dropping tag: " << tagPtr->getId()
                             << " due to small size: " << sz);
          continue;
        }
        sortedFactors.insert(MMap::value_type(sz, fac));
        writeTagCorners(t, cam->getIndex(), tagPtr, corners);
        tagsFound.insert(tag.id);
      } else {
        LOG_ERROR("dropping DUPLICATE TAG: " << tag.id);
      }
    }
    std::stringstream ss;
    for (const auto & tag : tagMsgs[i]->detections) {
      ss << " " << tag.id;
    }
    LOG_INFO(
      "frame " << frameNum_ << " [" << t << "] cam: " << cam->getName()
               << " sees tags: " << ss.str());
  }
  for (auto it = sortedFactors.rbegin(); it != sortedFactors.rend(); ++it) {
    factors->push_back(it->second);
  }
}

void TagSLAM::remapAndSquash(
  uint64_t t, std::vector<TagArrayConstPtr> * remapped,
  const std::vector<TagArrayConstPtr> & orig)
{
  //
  // Sometimes there are tags with duplicate ids in the data set.
  // In this case, remap the tag ids of the detected tags dependent
  // on time stamp, to something else so they become unique.
  for (size_t i = 0; i < orig.size(); i++) {
    const string & camName = cameras_[i]->getName();
    const auto it = camSquash_.find(camName);
    const std::set<int> * sqc = (it != camSquash_.end()) ? &(it->second) : NULL;
    const auto & o = orig[i];
    TagArrayPtr p(new TagArray());
    p->header = o->header;
    p->header.stamp = rosTime(t);
    const auto sq = squash_[i].find(t);
    for (const auto & tag : o->detections) {
      if (tag.hamming > maxHammingDistance_) {
        LOG_WARN(
          "dropped tag " << tag.id << " with hamming dist: " << tag.hamming
                         << " > " << maxHammingDistance_);
        continue;
      }
      if (sq != squash_[i].end() && sq->second.count(tag.id) != 0) {
        LOG_INFO("time " << t << " squashed tag: " << tag.id);
      } else {
        if (!sqc || sqc->count(tag.id) == 0) {  // no camera squash?
          p->detections.push_back(tag);
        }
      }
    }
    // remap
    for (auto & tag : p->detections) {
      auto it = tagRemap_.find(tag.id);
      if (it != tagRemap_.end()) {
        for (const ReMap & r : it->second) {
          if (
            t >= r.startTime && t <= r.endTime &&
            (r.camera.empty() || (camName == r.camera))) {
            tag.id = r.remappedId;
          }
        }
      }
    }
    remapped->push_back(p);
  }
}

void TagSLAM::readRemap(const YAML::Node & config)
{
  if (!config["tag_id_remap"]) {
    return;
  }
  const auto remap = config["tag_id_remap"];
  if (remap.IsSequence()) {
    LOG_INFO("found remap map!");
    for (size_t i = 0; i < remap.size(); i++) {
      if (!remap[i].IsMap()) continue;
      int remapId = -1;
      std::vector<ReMap> remaps;
      for (const auto & rm : remap[i]) {
        if (rm.Tag() == "id") {
          remapId = rm.as<int>();
        }
        if (rm.Tag() == "remaps") {
          const auto re = rm;
          if (re.IsSequence()) {
            for (size_t j = 0; j < re.size(); j++) {
              auto a = re[j];
              if (!a.IsMap()) {
                continue;
              }
              ReMap r(
                a["remap_id"].as<int>(),
                yaml::parse<std::string>(a, "camera", ""),
                yaml::parse_time(a, "start_time", 0),
                yaml::parse_time(a, "end_time", 0));
              remaps.push_back(r);
            }
          }
        }
      }
      if (remapId >= 0) {
        LOG_INFO("found remapping for tag " << remapId);
        tagRemap_[remapId] = remaps;
      }
    }
  }
}

static void insert_into(
  std::map<uint64_t, std::set<int>> * map, uint64_t t,
  const std::set<int> & tags)
{
  if (map->count(t) == 0) {
    (*map)[t] = tags;
  } else {
    (*map)[t].insert(tags.begin(), tags.end());
  }
}

void TagSLAM::parseTimeSquash(
  const YAML::Node & sq, uint64_t t, const std::set<int> & tags)
{
  const string cam = yaml::parse<std::string>(sq, "camera", "");
  if (cam.empty()) {  // squash all from given camera
    for (size_t i = 0; i < cameras_.size(); i++) {
      insert_into(&squash_[i], t, tags);
    }
  } else {
    for (size_t i = 0; i < cameras_.size(); i++) {
      if (cameras_[i]->getName() == cam) {
        insert_into(&squash_[i], t, tags);
      }
    }
  }
}

void TagSLAM::readSquash(const YAML::Node & config)
{
  for (size_t i = 0; i < cameras_.size(); i++) {
    (void)i;
    squash_.push_back(std::map<uint64_t, std::set<int>>());
  }
  if (!config["squash"]) {
    return;
  }
  const auto squash = config["squash"];
  if (squash.IsSequence()) {
    for (size_t i = 0; i < squash.size(); i++) {
      try {
        auto sq = squash[i];
        const std::set<int> tags =
          yaml::parse_container<std::set<int>>(sq, "tags", std::set<int>());
        // try to parse as time squash first
        const uint64_t t = yaml::parse_time(sq, "time", 0);
        if (t != 0) {
          parseTimeSquash(sq, t, tags);
        } else {  // squash all tags for this camera
          const string cam = yaml::parse<std::string>(sq, "camera");
          camSquash_[cam] = tags;
        }
      } catch (const std::runtime_error & e) {
        BOMB_OUT("failed to parse squash number " << i << " e: " << e.what());
      }
    }
  }
}

void TagSLAM::writeTagCorners(
  uint64_t t, int camIdx, const TagConstPtr & tag, const Point * img_corners)
{
  for (size_t i = 0; i < 4; i++) {
    tagCornerFile_ << t << " " << tag->getId() << " " << camIdx << " " << i
                   << " " << img_corners[i].x << " " << img_corners[i].y
                   << std::endl;
  }
}

}  // namespace tagslam

RCLCPP_COMPONENTS_REGISTER_NODE(tagslam::TagSLAM)
