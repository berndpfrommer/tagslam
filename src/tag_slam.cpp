/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_slam.h"
#include "tagslam/logging.h"
#include "tagslam/geometry.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/body_defaults.h"
#include "tagslam/body.h"
#include "tagslam/odometry_processor.h"
#include "tagslam/yaml_utils.h"
#include "tagslam/xml.h"
#include "tagslam/graph_utils.h"
#include "tagslam/factor/distance.h"

#include <flex_sync/sync.h>

#include <cv_bridge/cv_bridge.h>
#include <rosbag/view.h>

#include <rosgraph_msgs/Clock.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <XmlRpcException.h>

#include <boost/range/irange.hpp>

#include <sstream>
#include <iomanip>
#include <cmath>

const int QSZ = 1000;

namespace tagslam {
  using Odometry = nav_msgs::Odometry;
  using OdometryConstPtr = nav_msgs::OdometryConstPtr;
  using Image = sensor_msgs::Image;
  using ImageConstPtr = sensor_msgs::ImageConstPtr;
  using CompressedImage = sensor_msgs::CompressedImage;
  using CompressedImageConstPtr = sensor_msgs::CompressedImageConstPtr;
  using boost::irange;
  using std::string;
  using std::setw;
  using std::fixed;
  using std::setprecision;
#define FMT(X, Y) fixed << setw(X) << setprecision(Y)

  static tf::Transform to_tftf(const Transform &tf) {
    tf::Transform ttf;
    tf::transformEigenToTF(tf, ttf);
    return (ttf);
  }

  static void write_time(std::ostream &o, const ros::Time &t) {
    o << t.sec << "." << std::setfill('0') << setw(9) << t.nsec
      << " " << std::setfill(' ');
  }

  static void write_vec(std::ostream &o, const Eigen::Vector3d &v) {
    for (const auto &i: irange(0, 3)) {
      o << " " << FMT(7, 3) << v(i);
    }
  }
  
  static double find_size_of_tag(const geometry_msgs::Point *imgCorn) {
    Eigen::Matrix<double, 4, 2> x;
    x << imgCorn[0].x, imgCorn[0].y, imgCorn[1].x, imgCorn[1].y,
      imgCorn[2].x, imgCorn[2].y, imgCorn[3].x, imgCorn[3].y;
    // shoelace formula
    const double A =
      fabs(x(0,0)*x(1,1) + x(1,0)*x(2,1) + x(2,0)*x(3,1) + x(3,0)*x(0,1)
           -x(1,0)*x(0,1) - x(2,0)*x(1,1) - x(3,0)*x(2,1) - x(0,0)*x(3,1));
    return (A);
  }

  const std::map<std::string, OptimizerMode> optModeMap = {
    {"full", SLOW},
    {"slow", SLOW},
    {"fast", FAST}
  };
 
  TagSlam::TagSlam(const ros::NodeHandle &nh) : nh_(nh) {
    initialGraph_.reset(new Graph());
    // Alias the graph to the initial graph. That way during startup,
    // all updates that are done on graph_, are also done on the initial
    // graph
    graph_ = initialGraph_;
    graph_->setVerbosity("SILENT");
  }

  void TagSlam::sleep(double dt) const {
    ros::WallTime tw0 = ros::WallTime::now();
    for (ros::WallTime t = ros::WallTime::now(); t-tw0 < ros::WallDuration(dt);
         t = ros::WallTime::now()) {
      ros::spinOnce();
      ros::WallTime::sleepUntil(t + (t-tw0));
    }
  }

  void TagSlam::readParams() {
    nh_.param<string>("outbag", outBagName_, "out.bag");
    nh_.param<double>("playback_rate", playbackRate_, 5.0);
    nh_.param<string>("output_directory", outDir_, ".");
    nh_.param<string>("bag_file", inBagFile_, "");
    nh_.param<string>("fixed_frame_id", fixedFrame_, "map");
    nh_.param<int>("max_number_of_frames", maxFrameNum_, 1000000);
    nh_.param<int>("sync_queue_size", syncQueueSize_, 100);
    nh_.param<bool>("write_debug_images", writeDebugImages_, false);
    nh_.param<bool>("publish_ack", publishAck_, false);
    nh_.param<bool>("has_compressed_images", hasCompressedImages_, false);
  }

  void TagSlam::testForOldLaunchParameters() {
    std::vector<string> oldParams =
      {"pixel_noise", "minimum_viewing_angle", "max_subgraph_error",
       "max_num_incremental_opt", "optimizer_mode"};
    for (const auto &p: oldParams) {
      if (nh_.hasParam(p)) {
        BOMB_OUT("error to specify " << p <<
                 " as launch param, must be in " << "tagslam.yaml file now");
      }
    }
  }

  bool TagSlam::initialize() {
    testForOldLaunchParameters();
    readParams();
    XmlRpc::XmlRpcValue config, camConfig, camPoses;
    nh_.getParam("tagslam_config", config);
    graphUpdater_.parse(config);
    const auto ommi = optModeMap.find(graphUpdater_.getOptimizerMode());
    if (ommi == optModeMap.end()) {
      BOMB_OUT("invalid optimizer mode: " << optimizerMode_);
    } else {
      graph_->getOptimizer()->setMode(ommi->second);
    }
    ROS_INFO_STREAM("optimizer mode: " << graphUpdater_.getOptimizerMode());

    readSquash(config);
    readRemap(config);
    readBodies(config);
    readGlobalParameters(config);
    nh_.getParam("cameras", camConfig);
    readCameras(camConfig);
    nh_.getParam("camera_poses", camPoses);
    readCameraPoses(camPoses);
    measurements_ = measurements::read_all(config, this);
    // apply measurements
    for (auto &m: measurements_) {
      m->addToGraph(graph_);
      m->tryAddToOptimizer();
    }
    if (!runOnline()) {
      clockPub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", QSZ);
    }
    replayService_ = nh_.advertiseService("replay", &TagSlam::replay, this);
    dumpService_ = nh_.advertiseService("dump", &TagSlam::dump, this);
    plotService_ = nh_.advertiseService("plot", &TagSlam::plot, this);
    if (publishAck_) {
      ackPub_	 = nh_.advertise<std_msgs::Header>("acknowledge", 1);
    }
    // optimize the initial setup if necessary
    graph_->optimize(0);
    // open output files
    tagCornerFile_.open("tag_corners.txt");
    // make a deep copy of the initial graph now
    graph_.reset(initialGraph_->clone());
    return (true);
  }

  void TagSlam::subscribe() {
    std::vector<std::vector<std::string>> topics = makeTopics();
    if (hasCompressedImages_) {
      subSyncCompressed_.reset(
        new flex_sync::SubscribingSync<TagArray, CompressedImage, Odometry>(
          nh_, topics, std::bind(&TagSlam::syncCallbackCompressed, this,
                                 std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3), 5));
    } else {
      subSync_.reset(
        new flex_sync::SubscribingSync<TagArray, Image, Odometry>(
          nh_, topics, std::bind(&TagSlam::syncCallback, this,
                                 std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3), 5));
    }
  }

  void TagSlam::run() {
    sleep(1.0); // give rviz time to connect
    // this plays back the data
    try {
      playFromBag(inBagFile_);
    } catch (const OptimizerException &e) {
      ROS_WARN_STREAM("Terminated with optimizer exception!");
      ROS_WARN_STREAM("Check all your input noise settings!");
      ROS_WARN_STREAM("No mixing of large and small noise, right???");
      throw (e);
    }
    std::cout.flush();
  }

  void TagSlam::doDump(bool optimize) {
    graphUpdater_.printPerformance();
    // do final optimization
    if (optimize) {
      profiler_.reset("finalOptimization");
      const double error = graph_->optimizeFull(true /*force*/);
      profiler_.record("finalOptimization");
      ROS_INFO_STREAM("final error: " << error);
    }
    graph_->printUnoptimized();
    for (auto &m: measurements_) {
      m->printUnused();
    }
    publishTransforms(times_.empty() ? ros::Time(0) :
                      *(times_.rbegin()), true);
    tagCornerFile_.flush();
    
    outBag_.open(outBagName_, rosbag::bagmode::Write);
    writeToBag_ = true;
    doReplay(0 /* playback at full speed */);
    writeToBag_ = false;
    outBag_.close();

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
    for (auto &m: measurements_) {
      m->writeDiagnostics();
    }
    profiler_.record("writeMeasurementDiagnostics");
    std::cout << profiler_ << std::endl;
    std::cout.flush();
  }

  void TagSlam::finalize(bool optimize) {
    doDump(optimize);
    tagCornerFile_.close();
    for (auto const &op: odomProcessors_) {
      op.finalize();
    }
  }

  void TagSlam::readBodies(XmlRpc::XmlRpcValue config) {
    // read body defaults first in case
    // bodies do not provide all parameters
    BodyDefaults::parse(config);

    // now read bodies
    bodies_ = Body::parse_bodies(config);
    for (const auto &body: bodies_) {
      graph_utils::add_body(graph_.get(), *body);
      // add associated tags as vertices
      for (const auto &tag: body->getTags()) {
        tagMap_.insert(TagMap::value_type(tag->getId(), tag));
      }
      if (!body->isStatic()) {
        if (body->getFakeOdomTranslationNoise() > 0 &&
            body->getFakeOdomRotationNoise() > 0) {
          useFakeOdom_ = true;
          ROS_INFO_STREAM("using fake odom for " << body->getName());
        }
        nonstaticBodies_.push_back(body);
        odomPub_.push_back(
          nh_.advertise<nav_msgs::Odometry>("odom/body_"+body->getName(),QSZ));
        
        trajectoryPub_.push_back(nh_.advertise<nav_msgs::Path>("path/body_"+body->getName(),QSZ));
        trajectory_.push_back(nav_msgs::Path());
      }
    }
  }

  void TagSlam::readGlobalParameters(XmlRpc::XmlRpcValue config) {
    const string defbody = xml::parse<std::string>(config, "default_body", "");
    if (defbody.empty()) {
      ROS_WARN_STREAM("no default body specified!");
    } else {
      for (auto &body: bodies_) {
        if (body->getName() == defbody) {
          defaultBody_ = body;
          if (defaultBody_->getDefaultTagSize() <= 0) {
            BOMB_OUT("body " << defbody << " must have default tag size!");
          }
          break;
        }
      }
      if (!defaultBody_) { BOMB_OUT("cannot find default body: " << defbody); }
    }
    amnesia_ = xml::parse<bool>(config, "amnesia", false);
    if (amnesia_) {
      ROS_INFO_STREAM("using amnesia!");
    }
    maxHammingDistance_ = xml::parse<int>(config, "max_hamming_distance", 100);
  }


  void TagSlam::readCameraPoses(XmlRpc::XmlRpcValue config) {
    std::map<BodyConstPtr, int> numKnownCamPoses;
    for (auto &cam: cameras_) {
      if (numKnownCamPoses.count(cam->getRig()) == 0) {
        numKnownCamPoses.emplace(cam->getRig(), 0);
      }
      PoseWithNoise pwn; // defaults to invalid pose
      if (config.hasMember(cam->getName())) {
        pwn = xml::parse<PoseWithNoise>(config[cam->getName()], "pose",
                                        PoseWithNoise());
        if (pwn.isValid()) {
          numKnownCamPoses[cam->getRig()]++;
          ROS_INFO_STREAM("camera " << cam->getName() << " has known pose!");
        }
      }
      graph_utils::add_pose_maybe_with_prior(
        graph_.get(), ros::Time(0),
        Graph::cam_name(cam->getName()),pwn, true);
    }
    for (const auto &ncp: numKnownCamPoses) {
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

  void TagSlam::readCameras(XmlRpc::XmlRpcValue config) {
    if (config.getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
      BOMB_OUT("no camera configurations found!");
    }
    cameras_ = Camera::parse_cameras(config);
    ROS_INFO_STREAM("found " << cameras_.size() << " cameras");
    for (auto &cam: cameras_) {
      for (const auto &body: bodies_) {
        if (body->getName() == cam->getRigName()) {
          cam->setRig(body);
        }
      }
      if (!cam->getRig()) {
        BOMB_OUT("rig body not found: " << cam->getRigName());
      }
    }
  }

  template <typename T>
  static void process_images(const std::vector<T> &msgvec,
                             std::vector<cv::Mat> *images) {
    images->clear();
    for (const auto i: irange(0ul, msgvec.size())) {
      const auto &img = msgvec[i];
      cv::Mat im =
        cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::BGR8)->image;
      images->push_back(im);
    }
  }

  void TagSlam::testIfInBag(
    rosbag::Bag *bag, const std::vector<std::vector<string>> &topics) const {
  }

  std::vector<std::vector<std::string>>
  TagSlam::makeTopics() const {
    std::vector<std::vector<string>> topics(3);
    for (const auto &body: bodies_) {
      if (!body->getOdomTopic().empty()) {
        topics[2].push_back(body->getOdomTopic());
      }
    }
    for (const auto &cam: cameras_) {
      if (cam->getTagTopic().empty()) {
        BOMB_OUT("camera " << cam->getName() << " no tag topic!");
      }
      topics[0].push_back(cam->getTagTopic());
      if (writeDebugImages_) {
        if (cam->getImageTopic().empty()) {
          BOMB_OUT("camera " << cam->getName() << " no image topic!");
        }
        topics[1].push_back(cam->getImageTopic());
      }
    }
    ROS_INFO_STREAM("number of tag   topics: " << topics[0].size());
    ROS_INFO_STREAM("number of image topics: " << topics[1].size());
    ROS_INFO_STREAM("number of odom  topics: " << topics[2].size());
    return (topics);
  }

  bool TagSlam::plot(std_srvs::Trigger::Request& req,
                     std_srvs::Trigger::Response &res) {
    ROS_INFO_STREAM("plotting!");
    graph_utils::plot("graph.dot", graph_.get());
    res.message = "dump complete!";
    res.success = true;
    ROS_INFO_STREAM("finished dumping.");
    return (true);
  }
 
  bool TagSlam::replay(std_srvs::Trigger::Request& req,
                       std_srvs::Trigger::Response &res) {
    ROS_INFO_STREAM("replaying!");
    doReplay(playbackRate_);
    res.message = "replayed " + std::to_string(times_.size());
    res.success = true;
    ROS_INFO_STREAM("finished replaying " << times_.size() << " frames");
    return (true);
  }
  
  bool TagSlam::dump(std_srvs::Trigger::Request& req,
                     std_srvs::Trigger::Response &res) {
    ROS_INFO_STREAM("dumping!");
    doDump(true);
    res.message = "dump complete!";
    res.success = true;
    ROS_INFO_STREAM("finished dumping.");
    return (true);
  }

  void TagSlam::doReplay(double rate) {
    ros::WallRate wallRate(std::max(rate, 1e-3));
    for (const auto &t: times_) {
      publishAll(t);
      if (rate > 0) {
        wallRate.sleep();
      }
    }
    if (!times_.empty()) {
      publishTransforms(*times_.rbegin(), true);
    }
  }

  void TagSlam::publishCameraTransforms(const ros::Time &t,
                                         tf::tfMessage *tfMsg) {
    for (const auto &cam: cameras_) {
      Transform camTF;
      geometry_msgs::TransformStamped tfm;
      if (graph_utils::get_optimized_pose(*graph_, *cam, &camTF)) {
        const string &rigFrameId = cam->getRig()->getFrameId();
        auto ctf = tf::StampedTransform(
          to_tftf(camTF), t, rigFrameId, cam->getFrameId());
        tfBroadcaster_.sendTransform(ctf);
        tf::transformStampedTFToMsg(ctf, tfm);
        tfMsg->transforms.push_back(tfm);
        //ROS_DEBUG_STREAM("published transform for cam: " << cam->getName());
      }
    }
  }

  void TagSlam::publishTagAndBodyTransforms(const ros::Time &t,
                                             tf::tfMessage *tfMsg) {
    geometry_msgs::TransformStamped tfm;
    for (const auto &body: bodies_) {
      Transform bodyTF;
      const string &bodyFrameId = body->getFrameId();
      const ros::Time ts = body->isStatic() ? ros::Time(0) : t;
      if (graph_utils::get_optimized_pose(*graph_, ts, *body, &bodyTF)) {
        tf::StampedTransform btf = tf::StampedTransform(
          to_tftf(bodyTF), t, fixedFrame_, bodyFrameId);
        tf::transformStampedTFToMsg(btf, tfm);
        tfMsg->transforms.push_back(tfm);
        tfBroadcaster_.sendTransform(btf);
        for (const auto &tag: body->getTags()) {
          Transform tagTF;
          if (graph_utils::get_optimized_pose(*graph_, *tag, &tagTF)) {
            const std::string frameId = "tag_" + std::to_string(tag->getId());
            auto ttf = tf::StampedTransform(
              to_tftf(tagTF), t, bodyFrameId, frameId);
            tfBroadcaster_.sendTransform(ttf);
            tf::transformStampedTFToMsg(ttf, tfm);
            tfMsg->transforms.push_back(tfm);
          }
        }
      }
    }
  }

  void TagSlam::publishOriginalTagTransforms(const ros::Time &t,
                                              tf::tfMessage *tfMsg) {
    geometry_msgs::TransformStamped tfm;
    for (const auto &body: bodies_) {
      if (!body->getPoseWithNoise().isValid()) {
        continue;
      }
      const string &bodyFrameId = body->getFrameId();
      for (const auto &tag: body->getTags()) {
        if (tag->getPoseWithNoise().isValid()) {
          const Transform tagTF = tag->getPoseWithNoise().getPose();
          const std::string frameId = "o_tag_" + std::to_string(tag->getId());
          auto ttf =
            tf::StampedTransform(to_tftf(tagTF), t, bodyFrameId, frameId);
          tfBroadcaster_.sendTransform(ttf);
          tf::transformStampedTFToMsg(ttf, tfm);
          tfMsg->transforms.push_back(tfm);
        }
      }
    }
  }

  void TagSlam::publishTransforms(const ros::Time &t, bool orig) {
    tf::tfMessage tfMsg;
    publishTagAndBodyTransforms(t, &tfMsg);
    if (orig) {
      publishOriginalTagTransforms(t, &tfMsg);
    }
    publishCameraTransforms(t, &tfMsg);
    if (t != ros::Time(0) && writeToBag_) {
      // rosbag API does not like t == 0;
      outBag_.write<tf::tfMessage>("/tf", t, tfMsg);
    }
  }

  void TagSlam::playFromBag(const string &fname) {
    rosbag::Bag bag;
    ROS_INFO_STREAM("reading from bag: " << fname);
    bag.open(fname, rosbag::bagmode::Read);
    std::vector<std::vector<string>> topics = makeTopics();
    testIfInBag(&bag, topics);
    std::vector<string> flatTopics;
    for (const auto &v: topics) {
      flatTopics.insert(flatTopics.begin(), v.begin(), v.end());
    }
    double deltaStartTime{0};
    nh_.param<double>("bag_start_time", deltaStartTime, 0.0);
    rosbag::View dummyView(bag);
    const ros::Time startTime =
      dummyView.getBeginTime() + ros::Duration(deltaStartTime);
    publishTransforms(startTime, false);

    rosbag::View view(bag, rosbag::TopicQuery(flatTopics), startTime);
    ros::WallTime t0 = ros::WallTime::now();
    if (hasCompressedImages_) {
      std::function<void(const std::vector<TagArray::ConstPtr> &,
                         const std::vector<CompressedImage::ConstPtr> &,
                         const std::vector<Odometry::ConstPtr> &)> cb =
        std::bind(&TagSlam::syncCallbackCompressed, this,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3);
      processBag<TagArray, CompressedImage, Odometry>(
        &view, topics, cb);
    } else {
      std::function<void(const std::vector<TagArray::ConstPtr> &,
                         const std::vector<Image::ConstPtr> &,
                         const std::vector<Odometry::ConstPtr> &)> cb =
        std::bind(&TagSlam::syncCallback, this,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3);
      processBag<TagArray, Image, Odometry>(&view, topics, cb);
    }
    bag.close();
    ROS_INFO_STREAM("done processing bag, total wall time: " <<
                    (ros::WallTime::now() - t0).toSec());
  }

  void TagSlam::syncCallback(
    const std::vector<TagArrayConstPtr> &msgvec1,
    const std::vector<ImageConstPtr> &msgvec2,
    const std::vector<OdometryConstPtr> &msgvec3) {
    profiler_.reset("processImages");
    process_images<ImageConstPtr>(msgvec2, &images_);
    profiler_.record("processImages");
    profiler_.reset("processTagsAndOdom");
    processTagsAndOdom(msgvec1, msgvec3);
    profiler_.record("processTagsAndOdom");
  }
  
  void TagSlam::syncCallbackCompressed(
    const std::vector<TagArrayConstPtr> &msgvec1,
    const std::vector<CompressedImageConstPtr> &msgvec2,
    const std::vector<OdometryConstPtr> &msgvec3) {
    profiler_.reset("processCompressedImages");
    process_images<CompressedImageConstPtr>(msgvec2, &images_);
    profiler_.record("processCompressedImages");
    profiler_.reset("processTagsAndOdom");
    processTagsAndOdom(msgvec1, msgvec3);
    profiler_.record("processTagsAndOdom");
  }

  static nav_msgs::Odometry make_odom(const ros::Time &t,
                                      const std::string &fixed_frame,
                                      const std::string &child_frame,
                                      const Transform &pose) {
    nav_msgs::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = fixed_frame;
    odom.child_frame_id = child_frame;
    tf::poseEigenToMsg(pose, odom.pose.pose);
    return (odom);
  }

  void TagSlam::publishBodyOdom(const ros::Time &t) {
    for (const auto body_idx: irange(0ul, nonstaticBodies_.size())) {
      const auto body = nonstaticBodies_[body_idx];
      Transform pose;
      if (graph_utils::get_optimized_pose(*graph_, t, *body, &pose)) {
        auto msg = make_odom(t, fixedFrame_, body->getOdomFrameId(), pose);
        odomPub_[body_idx].publish(msg);
        if (writeToBag_) {
          outBag_.write<nav_msgs::Odometry>(
            "/tagslam/odom/body_" + body->getName(), t, msg);
        }
        
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = t;
        pose_msg.header.frame_id = body->getOdomFrameId();
        pose_msg.pose = msg.pose.pose;

        trajectory_[body_idx].header.stamp = t;
        trajectory_[body_idx].header.frame_id = fixedFrame_;
        trajectory_[body_idx].poses.push_back(pose_msg);
        trajectoryPub_[body_idx].publish(trajectory_[body_idx]);
      }
    }
  }

  bool TagSlam::anyTagsVisible(const std::vector<TagArrayConstPtr> &tagmsgs) {
    for (const auto &msg: tagmsgs) {
      if (!findTags(msg->apriltags).empty()) {
        return (true);
      }
    }
    return (false);
  }

  void TagSlam::copyPosesAndReset() {
    //
    // For all dynamic bodies, copy the previous pose
    // and bolt it down with a pose prior. In combination
    // with fake odometry, this helps initialization.
    //
    ros::Time t = times_.back();
    Graph *g = initialGraph_->clone();
    for (const auto &body: bodies_) {
      if (body->isStatic() && body->getPoseWithNoise().isValid()) {
        continue;
      }
      Transform pose;
      if (graph_utils::get_optimized_pose(*graph_, t, *body, &pose)) {
        const std::string name = Graph::body_name(body->getName());
        // add pose to graph and optimizer
        const VertexDesc v = g->addPose(t, name, false /*isCamPose*/);
        const PoseValuePtr pp = std::dynamic_pointer_cast<value::Pose>((*g)[v]);
        pp->addToOptimizer(pose, g);
        // add pose prior to graph and optimizer
        const double ns = 0.001;
        PoseWithNoise pwn(pose, PoseNoise::make(ns, ns), true);
        AbsolutePosePriorFactorPtr
          app(new factor::AbsolutePosePrior(t, pwn, pp->getName()));
        app->addToGraph(app, g);
        app->addToOptimizer(g);
      }
    }
    times_.clear();
    times_.push_back(t); // such that fake odom works!
    graph_.reset(g); // now use the new graph
  }

  void TagSlam::processTagsAndOdom(
    const std::vector<TagArrayConstPtr> &origtagmsgs,
    const std::vector<OdometryConstPtr> &odommsgs) {
    profiler_.reset("processOdom");
    if (amnesia_ && !times_.empty()) {
      copyPosesAndReset();
    }
    std::vector<TagArrayConstPtr> tagmsgs;
    remapAndSquash(&tagmsgs, origtagmsgs);
    if (tagmsgs.empty() && odommsgs.empty()) {
      ROS_WARN_STREAM("neither tags nor odom!");
      return;
    }

    const auto &header = tagmsgs.empty() ?
      odommsgs[0]->header : tagmsgs[0]->header;
    const ros::Time t = header.stamp;
    const bool hasOdom = !odommsgs.empty() || useFakeOdom_;
    if (anyTagsVisible(tagmsgs) || hasOdom) {
      // if we have any new valid observations,
      // add unknown poses for all non-static bodies
      for (const auto &body: nonstaticBodies_) {
        graph_->addPose(t, Graph::body_name(body->getName()), false);
      }
    }
    std::vector<VertexDesc> factors;
    if (odommsgs.size() != 0) {
      processOdom(odommsgs, &factors);
    } else if (useFakeOdom_) {
      fakeOdom(t, &factors);
    }
    profiler_.record("processOdom");
    profiler_.reset("processTags");
    processTags(tagmsgs, &factors);
    profiler_.record("processTags");
    profiler_.reset("processNewFactors");
    try {
      graphUpdater_.processNewFactors(graph_.get(), t, factors);
    } catch (const OptimizerException &e) {
      ROS_WARN_STREAM("optimizer crapped out!");
      ROS_WARN_STREAM(e.what());
      finalize(false);
      throw (e);
    }
    profiler_.record("processNewFactors");
    profiler_.reset("publish");
    times_.push_back(t);
    publishAll(t);
    frameNum_++;
    if (publishAck_) {
      ackPub_.publish(header);
    }
    profiler_.record("publish");
    if (runOnline() && frameNum_ >= maxFrameNum_) {
      if (publishAck_) {
        auto h = header;
        h.frame_id = "FINISHED!";
        ackPub_.publish(h);
      }
      ROS_INFO_STREAM("reached max number of frames, finished!");
      finalize(true);
      ros::shutdown();
    }
  }

  void TagSlam::publishAll(const ros::Time &t) {
    publishBodyOdom(t);
    if (!runOnline()) {
      rosgraph_msgs::Clock clockMsg;
      clockMsg.clock = t;
      clockPub_.publish(clockMsg);
    }
    publishTransforms(t, false);
  }

  void TagSlam::fakeOdom(const ros::Time &tCurr,
                          std::vector<VertexDesc> *factors) {
    if (!times_.empty()) {
      const auto &tPrev = times_.back();
      for (const auto &body: bodies_) {
        if (!body->isStatic()) {
          const PoseNoise pn =
            PoseNoise::make(body->getFakeOdomRotationNoise(),
                             body->getFakeOdomTranslationNoise());
          const PoseWithNoise pwn(Transform::Identity(), pn, true);
          factors->push_back(OdometryProcessor::add_body_pose_delta(
                               graph_.get(), tPrev, tCurr, body, pwn));
        }
      }
    }
  }

  
  void TagSlam::setupOdom(const std::vector<OdometryConstPtr> &odomMsgs) {
    std::set<BodyConstPtr> bodySet;
    for (const auto odomIdx: irange(0ul, odomMsgs.size())) {
      const auto &frameId = odomMsgs[odomIdx]->child_frame_id;
      BodyConstPtr bpt;
      for (const auto &body: bodies_) {
        if (body->getOdomFrameId() == frameId) {
          bpt = body;
        }
      }
      if (!bpt) {
        BOMB_OUT("no body found for odom frame id: " << frameId);
      }
      if (bodySet.count(bpt) != 0) {
        ROS_WARN_STREAM("multiple bodies with frame id: " << frameId);
        ROS_WARN_STREAM("This will screw up the odom!");
      }
      ROS_INFO_STREAM("have odom from " << bpt->getName() << " " << frameId);
      odomProcessors_.push_back(OdometryProcessor(nh_, bpt));
    }
  }

  void
  TagSlam::processOdom(const std::vector<OdometryConstPtr> &odomMsgs,
                        std::vector<VertexDesc> *factors) {
    if (odomProcessors_.empty()) {
      setupOdom(odomMsgs);
    }
    // from odom child frame id, deduce bodies
    for (const auto odomIdx: irange(0ul, odomMsgs.size())) {
      const auto &msg = odomMsgs[odomIdx];
      odomProcessors_[odomIdx].process(graph_.get(), msg, factors);
      //graph_.test();
    }
  }

  TagPtr TagSlam::addTag(int tagId, const BodyPtr &body) const {
    TagPtr p;
    if (body->ignoreTag(tagId)) {
      return (p);
    }
    p = Tag::make(tagId, 6 /*num bits = tag family */,
                  body->getDefaultTagSize(),
                  PoseWithNoise() /* invalid pose */, body);
    body->addTag(p);
    ROS_INFO_STREAM("new tag " << tagId << " attached to " <<
                    body->getName());
    graph_utils::add_tag(graph_.get(), *p);
    return (p);
  }
  
  TagConstPtr TagSlam::findTag(int tagId) {
    TagMap::iterator it = tagMap_.find(tagId);
    TagPtr p;
    if (it == tagMap_.end()) {
      if (!defaultBody_) {
        ROS_WARN_STREAM("no default body, ignoring tag: " << tagId);
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
  
  void TagSlam::writeCameraPoses(const string &fname) const {
    std::ofstream f(fname);
    for (const auto &cam : cameras_) {
      f << cam->getName() << ":" << std::endl;
      try {
        PoseWithNoise pwn = graph_utils::get_optimized_pose_with_noise(
          *graph_, Graph::cam_name(cam->getName()));
        if (pwn.isValid()) {
          f << "  pose:" << std::endl;
          yaml_utils::write_pose_with_covariance(f, "    ", pwn.getPose(),
                                                 pwn.getNoise());
        }
      } catch (const OptimizerException &e) {
        ROS_WARN_STREAM("no optimized pose for: " << cam->getName());
      }
    }
  }

  void TagSlam::writeFullCalibration(const string &fname) const {
    std::ofstream f(fname);
    for (const auto &cam : cameras_) {
      Transform tf;
      if (graph_utils::get_optimized_pose(
            *graph_, ros::Time(0), Graph::cam_name(cam->getName()), &tf)) {
        f << cam->getName() << ":" << std::endl;
        f << "  T_cam_body:" << std::endl;
        yaml_utils::write_matrix(f, "  ", tf.inverse());
        cam->getIntrinsics().writeYaml(f, "  ");
      } else {
        ROS_WARN_STREAM("no pose found for camera: " << cam->getName());
      }
    }
  }

  void TagSlam::writePoses(const string &fname) const {
    std::ofstream f(fname);
    f << "bodies:" << std::endl;
    const std::string idn = "       ";
    for (const auto &body: bodies_) {
      Transform bodyTF;
      body->write(f, " ");
      if (body->isStatic()) {
        try {
          PoseWithNoise pwn = graph_utils::get_optimized_pose_with_noise(
            *graph_, Graph::body_name(body->getName()));
          if (pwn.isValid()) {
            f << "     pose:" << std::endl;
            yaml_utils::write_pose(f, "       ", pwn.getPose(),
                                   pwn.getNoise(), true);
          }
        } catch (const OptimizerException &e) {
          ROS_WARN_STREAM("cannot find pose for " << body->getName());
        }
      }
      if (body->printTags()) {
        f << "     tags:" << std::endl;
        for (const auto &tag: body->getTags()) {
          Transform tagTF;
          try {
            if (graph_utils::get_optimized_pose(*graph_, *tag, &tagTF)) {
              f << idn << "- id: "   << tag->getId() << std::endl;
              f << idn << "  size: " << tag->getSize() << std::endl;
              f << idn << "  pose:" << std::endl;
              const auto &pwn = tag->getPoseWithNoise();
              yaml_utils::write_pose(f, idn + "    ", tagTF, pwn.getNoise(),
                                     true);
            }
          } catch (const OptimizerException &e) {
            ROS_WARN_STREAM("cannot find pose for tag " << tag->getId());
          }
        }
      }
    }
  }

  void TagSlam::writeTimeDiagnostics(const string &fname) const {
    std::ofstream f(fname);
    const Graph::TimeToErrorMap m = graph_->getTimeToErrorMap();
    for (const auto &te: m) {
      write_time(f, te.first);
      double err(0);
      for (const auto fe: te.second) {
        err += fe.second;
      }
      f << err;
      for (const auto fe: te.second) {
        f << " " << (*fe.first) << ":err=" << fe.second;
      }
      f << std::endl;
    }
  }

  void TagSlam::writeErrorMap(const string &fname) const {
    std::ofstream f(fname);
    const auto errMap = graph_->getErrorMap();
    for (const auto &v: errMap) {
      const auto &vp = graph_->getVertex(v.second);
      f << FMT(8,3) << v.first << " ";
      write_time(f, vp->getTime());
      f << " " <<  *vp << std::endl;
    }
  }

  void TagSlam::writeTagDiagnostics(const string &fname) const {
    std::ofstream f(fname);
    const std::string idn = "       ";
    for (const auto &body: bodies_) {
      for (const auto &tag: body->getTags()) {
        Transform tagTF;
        if (graph_utils::get_optimized_pose(*graph_, *tag, &tagTF)) {
          const auto &pwn = tag->getPoseWithNoise();
          if (pwn.isValid()) {
            const Transform poseDiff = pwn.getPose().inverse() * tagTF;
            const auto x = poseDiff.translation();
            Eigen::AngleAxisd aa;
            aa.fromRotationMatrix(poseDiff.rotation());
            f << setw(3) << tag->getId() << " " << FMT(6,3) << x.norm();
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
  
  std::vector<TagConstPtr>
  TagSlam::findTags(const std::vector<Apriltag> &ta) {
    std::vector<TagConstPtr> tpv;
    for (const auto &tag: ta) {
      TagConstPtr tagPtr = findTag(tag.id);
      if (tagPtr) {
        tpv.push_back(tagPtr);
      } else {
        ROS_INFO_STREAM("ignoring tag as requested in config: " << tag.id);
      }
    }
    return (tpv);
  }

  void TagSlam::processTags(const std::vector<TagArrayConstPtr> &tagMsgs,
                             std::vector<VertexDesc> *factors) {
    if (tagMsgs.size() != cameras_.size()) {
      BOMB_OUT("tag msgs size mismatch: " << tagMsgs.size()
               << " " << cameras_.size());
    }
    typedef std::multimap<double, VertexDesc> MMap;
    MMap sortedFactors;

    for (const auto i: irange(0ul, cameras_.size())) {
      const ros::Time &t = tagMsgs[i]->header.stamp;
      const auto &cam = cameras_[i];
      const auto tags = findTags(tagMsgs[i]->apriltags);
      if (!tags.empty()) {
        // insert time-dependent camera pose
        graph_->addPose(t, Graph::cam_name(cam->getName()),
                        true /*camPose*/);
        // and tie it to the time-independent camera pose
        // with a relative prior
        const PoseWithNoise pn(Transform::Identity(), cam->getWiggle(), true);
        string  name = Graph::cam_name(cam->getName());
        RelativePosePriorFactorPtr
          fac(new factor::RelativePosePrior(t, ros::Time(0), pn, name));
        VertexDesc v = fac->addToGraph(fac, graph_.get());
        sortedFactors.insert(MMap::value_type(1e10, v));
      }
      for (const auto &tag: tagMsgs[i]->apriltags) {
        TagConstPtr tagPtr = findTag(tag.id);
        if (tagPtr) {
          const geometry_msgs::Point *corners = &(tag.corners[0]);
          TagProjectionFactorPtr fp(
            new factor::TagProjection(t, cam, tagPtr, corners,
                                      graphUpdater_.getPixelNoise(),
                                      cam->getName() + "-" +
                                      Graph::tag_name(tagPtr->getId())));
          auto fac = fp->addToGraph(fp, graph_.get());
          double sz = find_size_of_tag(corners);
          sortedFactors.insert(MMap::value_type(sz, fac));
          writeTagCorners(t, cam->getIndex(), tagPtr, corners);
        }
      }
      std::stringstream ss;
      for (const auto &tag: tagMsgs[i]->apriltags) {
        ss << " " << tag.id;
      }
      ROS_INFO_STREAM("frame " << frameNum_ << " " << cam->getName()
                      << " sees tags: " << ss.str());
    }
    for (auto it = sortedFactors.rbegin(); it != sortedFactors.rend(); ++it) {
      factors->push_back(it->second);
    }
  }

  void TagSlam::remapAndSquash(std::vector<TagArrayConstPtr> *remapped,
                                const std::vector<TagArrayConstPtr> &orig) {
    //
    // Sometimes there are tags with duplicate ids in the data set.
    // In this case, remap the tag ids of the detected tags dependent
    // on time stamp, to something else so they become unique.
    for (const auto i: irange(0ul, orig.size())) {
      const string &camName = cameras_[i]->getName();
      const auto it = camSquash_.find(camName);
      const std::set<int> *sqc = (it != camSquash_.end()) ?
        &(it->second) : NULL;
      const auto &o = orig[i];
      const ros::Time t = o->header.stamp;
      TagArrayPtr p(new TagArray());
      p->header = o->header;
      const auto sq = squash_.find(t);
      for (const auto &tag: o->apriltags) {
        if (tag.hamming > maxHammingDistance_) {
          ROS_WARN_STREAM("dropped tag " << tag.id <<
                          " with hamming dist: " << tag.hamming
                          << " > " << maxHammingDistance_);
          continue;
        }
        if (sq != squash_.end() && sq->second.count(tag.id) != 0) {
          ROS_INFO_STREAM("time " << t << " squashed tag: " <<  tag.id);
        } else{
          if (!sqc || sqc->count(tag.id) == 0) { // no camera squash?
            p->apriltags.push_back(tag);
          }
        }
      }
      // remap 
      for (auto &tag: p->apriltags) {
        auto it = tagRemap_.find(tag.id);
        if (it != tagRemap_.end()) {
          for (const ReMap &r: it->second) {
            if (t >= r.startTime && t <= r.endTime &&
                (r.camera.empty() || (camName == r.camera))) {
              tag.id = r.remappedId;
            }
          }
        }
      }
      remapped->push_back(p);
    }
  }

  void TagSlam::readRemap(XmlRpc::XmlRpcValue config) {
    if (!config.hasMember("tag_id_remap")) {
      return;
    }
    XmlRpc::XmlRpcValue remap = config["tag_id_remap"];
    if (remap.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      ROS_INFO_STREAM("found remap map!");
      for (const auto i: irange(0, remap.size())) {
        if (remap[i].getType() !=
            XmlRpc::XmlRpcValue::TypeStruct) continue;
        int remapId = -1;
        std::vector<ReMap> remaps;
        for (XmlRpc::XmlRpcValue::iterator it = remap[i].begin();
             it != remap[i].end(); ++it) {
          if (it->first == "id") {
            remapId = static_cast<int>(it->second);
          }
          if (it->first == "remaps") {
            const auto re = it->second;
            if (re.getType() == XmlRpc::XmlRpcValue::TypeArray) {
              for (const auto j: irange(0, re.size())) {
                auto a = re[j];
                if (a.getType() !=
                    XmlRpc::XmlRpcValue::TypeStruct) continue;
                ReMap r(static_cast<int>(a["remap_id"]),
                        xml::parse<std::string>(a, "camera", ""),
                        ros::Time(static_cast<double>(a["start_time"])),
                        ros::Time(static_cast<double>(a["end_time"])));
                remaps.push_back(r);
              }
            }
          }
        }
        if (remapId >= 0) {
          ROS_INFO_STREAM("found remapping for tag " << remapId);
          tagRemap_[remapId] = remaps;
        }
      }
    }
  }

  void TagSlam::readSquash(XmlRpc::XmlRpcValue config) {
    if (!config.hasMember("squash")) {
      return;
    }
    XmlRpc::XmlRpcValue squash = config["squash"];
    if (squash.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (const auto i: irange(0, squash.size())) {
        try {
          auto sq = squash[i];
          const std::set<int> tags =
            xml::parse_container<std::set<int>>(sq, "tags", std::set<int>());
          // try to parse as time squash
          const ros::Time t = xml::parse<ros::Time>(sq, "time", ros::Time(0));
          if (t != ros::Time(0)) {
            squash_[t] = tags;
          } else {
            const string cam = xml::parse<std::string>(sq, "camera");
            camSquash_[cam] = tags;
          }
        } catch (const XmlRpc::XmlRpcException &e) {
          BOMB_OUT("failed to parse squash number " << i);
        }
      }
    }
  }
  
  void
  TagSlam::writeTagCorners(const ros::Time &t, int camIdx,
                            const TagConstPtr &tag,
                            const geometry_msgs::Point *img_corners) {
    for (const auto i: irange(0, 4)) {
      tagCornerFile_ << t << " " << tag->getId() << " " << camIdx << " "
                     << i << " " << img_corners[i].x << " "
                     << img_corners[i].y << std::endl;
    }
  }


}  // end of namespace

