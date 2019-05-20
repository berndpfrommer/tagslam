/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_slam2.h"
#include "tagslam/logging.h"
#include "tagslam/geometry.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/body_defaults.h"
#include "tagslam/body.h"
#include "tagslam/odometry_processor.h"
#include "tagslam/yaml_utils.h"
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
 
  TagSlam2::TagSlam2(const ros::NodeHandle &nh) : nh_(nh) {
    graph_.reset(new Graph());
    graph_->setVerbosity("SILENT");
    graphUpdater_.setGraph(graph_);
  }

  void TagSlam2::sleep(double dt) const {
    ros::WallTime tw0 = ros::WallTime::now();
    for (ros::WallTime t = ros::WallTime::now(); t-tw0 < ros::WallDuration(dt);
         t = ros::WallTime::now()) {
      ros::spinOnce();
      ros::WallTime::sleepUntil(t + (t-tw0));
    }
  }

  void TagSlam2::readParams() {
    nh_.param<string>("outbag", outBagName_, "out.bag");
    nh_.param<double>("playback_rate", playbackRate_, 5.0);
    nh_.param<double>("pixel_noise", pixelNoise_, 1.0);
    nh_.param<string>("camera_poses_out_file",
                      camPoseFile_, "camera_poses.yaml");
    nh_.param<string>("poses_out_file",
                      poseFile_, "poses.yaml");
    nh_.param<string>("bag_file", inBagFile_, "");
    nh_.param<string>("fixed_frame_id", fixedFrame_, "map");
    nh_.param<int>("max_number_of_frames", maxFrameNum_, 1000000);
    nh_.param<bool>("write_debug_images", writeDebugImages_, false);
    nh_.param<bool>("has_compressed_images", hasCompressedImages_, false);
    
    // --- graph updater params ---
    
    bool   optFullGraph;
    int    maxIncOpt;
    double maxSubgraphError, angleLimit;
    nh_.param<bool>("optimize_full_graph", optFullGraph, false);
    nh_.param<double>("minimum_viewing_angle", angleLimit, 20);
    graphUpdater_.setMinimumViewingAngle(angleLimit);
    nh_.param<double>("max_subgraph_error", maxSubgraphError, 50.0);
    graphUpdater_.setMaxSubgraphError(maxSubgraphError);
    nh_.param<int>("max_num_incremental_opt", maxIncOpt, 100);
    graphUpdater_.setMaxNumIncrementalOpt(maxIncOpt);
    graphUpdater_.setOptimizeFullGraph(optFullGraph);
  }

  bool TagSlam2::initialize() {
    readParams();
    XmlRpc::XmlRpcValue config, camConfig;
    nh_.getParam("tagslam_config", config);
    readSquash(config);
    readRemap(config);
    readBodies(config);
    readDefaultBody(config);
    if (nh_.getParam("cameras", camConfig)) {
      readCameras(camConfig);
    } else {
      BOMB_OUT("no cameras config found!");
    }
    measurements_ = measurements::read_all(config, this);
    // apply measurements
    for (auto &m: measurements_) {
      m->addToGraph(graph_);
      m->tryAddToOptimizer();
    }
    clockPub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", QSZ);
    service_ = nh_.advertiseService("replay", &TagSlam2::replay, this);
    // optimize the initial setup if necessary
    graph_->optimize(0);
    // open output files
    outBag_.open(outBagName_, rosbag::bagmode::Write);
    tagCornerFile_.open("tag_corners.txt");
    return (true);
  }

  void TagSlam2::subscribe() {
    std::vector<std::vector<std::string>> topics = makeTopics();
    if (hasCompressedImages_) {
      subSyncCompressed_.reset(
        new flex_sync::SubscribingSync<TagArray, CompressedImage, Odometry>(
          nh_, topics, std::bind(&TagSlam2::syncCallbackCompressed, this,
                                 std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3), 5));
    } else {
      subSync_.reset(
        new flex_sync::SubscribingSync<TagArray, Image, Odometry>(
          nh_, topics, std::bind(&TagSlam2::syncCallback, this,
                                 std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3), 5));
    }
    // subscribe to 
  }

  void TagSlam2::run() {
    sleep(1.0); // give rviz time to connect
    // this plays back the data
    playFromBag(inBagFile_);
    std::cout.flush();
  }

  void TagSlam2::finalize() {
    graphUpdater_.printPerformance();
    // do final optimization
    profiler_.reset();
    const double error = graph_->optimizeFull(true /*force*/);
    profiler_.record("finalOptimization");
    ROS_INFO_STREAM("final error: " << error);
    graph_->printUnoptimized();
    for (auto &m: measurements_) {
      m->printUnused();
    }
    publishTransforms(times_.empty() ? ros::Time(0) :
                      *(times_.rbegin()), true);
    outBag_.close();
    tagCornerFile_.close();
    
    writeCameraPoses(camPoseFile_);
    profiler_.reset();
    writePoses(poseFile_);
    profiler_.record("writePoses");
    writeErrorMap("error_map.txt");
    profiler_.record("writeErrorMaps");
    writeTagDiagnostics("tag_diagnostics.txt");
    profiler_.record("writeTagDiagnostics");
    writeTimeDiagnostics("time_diagnostics.txt");
    profiler_.record("writeTimeDiagnostics");
    for (auto &m: measurements_) {
      m->writeDiagnostics();
    }
    profiler_.record("writeDiagnostics");
    std::cout << profiler_ << std::endl;
    std::cout.flush();
  }

  void TagSlam2::readBodies(XmlRpc::XmlRpcValue config) {
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
      }
    }
  }

  void TagSlam2::readDefaultBody(XmlRpc::XmlRpcValue config) {
    try {
      const string defbody = config["default_body"];
      if (defbody.empty()) {
        ROS_WARN_STREAM("no default body specified!");
        return;
      }
      for (auto &body: bodies_) {
        if (body->getName() == defbody) {
          ROS_INFO_STREAM("default body: " << defbody);
          defaultBody_ = body;
        }
      }
      if (!defaultBody_) {
        ROS_WARN_STREAM("cannot find default body: " << defbody);
      }
    } catch (const XmlRpc::XmlRpcException &e) {
      ROS_WARN("no default body!");
    }
  }

  void TagSlam2::readCameras(XmlRpc::XmlRpcValue config) {
    cameras_ = Camera2::parse_cameras(config);
    ROS_INFO_STREAM("found " << cameras_.size() << " cameras");
    bool camHasKnownPose(false);
    for (auto &cam: cameras_) {
      for (const auto &body: bodies_) {
        if (body->getName() == cam->getRigName()) {
          cam->setRig(body);
        }
      }
      if (!cam->getRig()) {
        BOMB_OUT("rig body not found: " << cam->getRigName());
      }
      PoseWithNoise pwn = PoseWithNoise::parse(cam->getName(), nh_);
      if (pwn.isValid()) {
        camHasKnownPose = true;
        ROS_INFO_STREAM("camera " << cam->getName() << " has known pose!");
      }
      graph_utils::add_pose_maybe_with_prior(
        graph_.get(), ros::Time(0), Graph::cam_name(cam->getName()),pwn, true);
    }
    if (!camHasKnownPose) {
      BOMB_OUT("at least one camera must have known pose!");
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

  void TagSlam2::testIfInBag(
    rosbag::Bag *bag, const std::vector<std::vector<string>> &topics) const {
  }

  std::vector<std::vector<std::string>>
  TagSlam2::makeTopics() const {
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

  bool TagSlam2::replay(std_srvs::Trigger::Request& req,
                        std_srvs::Trigger::Response &res) {
    ROS_INFO_STREAM("replaying!");
    outBag_.open(outBagName_, rosbag::bagmode::Write);
    ros::Time t0(0);
    for (const auto &t: times_) {
      publishAll(t);
      if (t0 != ros::Time(0)) {
        sleep((t - t0).toSec() / playbackRate_);
      }
      t0 = t;
    }
    publishTransforms(t0, true);
    res.message = "replayed " + std::to_string(times_.size());
    res.success = true;
    ROS_INFO_STREAM("finished replaying " << times_.size());
    outBag_.close();
    return (true);
  }

  void TagSlam2::publishCameraTransforms(const ros::Time &t,
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

  void TagSlam2::publishTagAndBodyTransforms(const ros::Time &t, tf::tfMessage *tfMsg) {
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

  void TagSlam2::publishOriginalTagTransforms(const ros::Time &t,
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

  void TagSlam2::publishTransforms(const ros::Time &t, bool orig) {
    tf::tfMessage tfMsg;
    publishTagAndBodyTransforms(t, &tfMsg);
    if (orig) {
      publishOriginalTagTransforms(t, &tfMsg);
    }
    publishCameraTransforms(t, &tfMsg);
    outBag_.write<tf::tfMessage>("/tf", t, tfMsg);
  }

  void TagSlam2::playFromBag(const string &fname) {
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
    ros::Time tFinal;
    if (hasCompressedImages_) {
      flex_sync::Sync<TagArray, CompressedImage, Odometry>
        sync3c(topics, std::bind(&TagSlam2::syncCallbackCompressed, this,
                             std::placeholders::_1, std::placeholders::_2,
                             std::placeholders::_3));
      processBag(&sync3c, &view);
      bag.close();
    } else {
      flex_sync::Sync<TagArray, Image, Odometry>
        sync3(topics, std::bind(&TagSlam2::syncCallback, this,
                            std::placeholders::_1, std::placeholders::_2,
                            std::placeholders::_3));
      processBag(&sync3, &view);
      bag.close();
    }
    ROS_INFO_STREAM("done processing bag!");
  }

  void TagSlam2::syncCallback(
    const std::vector<TagArrayConstPtr> &msgvec1,
    const std::vector<ImageConstPtr> &msgvec2,
    const std::vector<OdometryConstPtr> &msgvec3) {
    process_images<ImageConstPtr>(msgvec2, &images_);
    processTagsAndOdom(msgvec1, msgvec3);
  }
  
  void TagSlam2::syncCallbackCompressed(
    const std::vector<TagArrayConstPtr> &msgvec1,
    const std::vector<CompressedImageConstPtr> &msgvec2,
    const std::vector<OdometryConstPtr> &msgvec3) {
    process_images<CompressedImageConstPtr>(msgvec2, &images_);
    processTagsAndOdom(msgvec1, msgvec3);
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

  void TagSlam2::publishBodyOdom(const ros::Time &t) {
    for (const auto body_idx: irange(0ul, nonstaticBodies_.size())) {
      const auto body = nonstaticBodies_[body_idx];
      Transform pose;
      if (graph_utils::get_optimized_pose(*graph_, t, *body, &pose)) {
        auto msg = make_odom(t, fixedFrame_, body->getOdomFrameId(), pose);
        odomPub_[body_idx].publish(msg);
        outBag_.write<nav_msgs::Odometry>(
          "odom/body_" + body->getName(), t, msg);
      }
    }
  }

  bool TagSlam2::anyTagsVisible(const std::vector<TagArrayConstPtr> &tagmsgs) {
    for (const auto &msg: tagmsgs) {
      if (!findTags(msg->apriltags).empty()) {
        return (true);
      }
    }
    return (false);
  }

  void TagSlam2::processTagsAndOdom(
    const std::vector<TagArrayConstPtr> &origtagmsgs,
    const std::vector<OdometryConstPtr> &odommsgs) {
    std::vector<TagArrayConstPtr> tagmsgs;
    remapAndSquash(&tagmsgs, origtagmsgs);
    if (tagmsgs.empty() && odommsgs.empty()) {
      ROS_ERROR_STREAM("neither tags nor odom!");
      return;
    }

    const ros::Time t = tagmsgs.empty() ?
      odommsgs[0]->header.stamp : tagmsgs[0]->header.stamp;
    const bool hasOdom = !odommsgs.empty() || useFakeOdom_;
    if (anyTagsVisible(tagmsgs) || hasOdom) {
      // if we have any new valid observations,
      // add unknown poses for all non-static bodies
      for (const auto &body: nonstaticBodies_) {
        graph_->addPose(t, Graph::body_name(body->getName()), false);
      }
    }
    std::vector<VertexDesc> factors;
    profiler_.reset();
    if (odommsgs.size() != 0) {
      processOdom(odommsgs, &factors);
    } else if (useFakeOdom_) {
      fakeOdom(t, &factors);
    }
    profiler_.record("processOdom");
    processTags(tagmsgs, &factors);
    profiler_.record("processTags");
    graphUpdater_.processNewFactors(t, factors);
    profiler_.record("processNewFactors");

    times_.push_back(t);
    publishAll(t);
    profiler_.record("publishAll");
    frameNum_++;
  }

  void TagSlam2::publishAll(const ros::Time &t) {
    publishBodyOdom(t);
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = t;
    clockPub_.publish(clockMsg);
    publishTransforms(t, false);
  }

  void TagSlam2::fakeOdom(const ros::Time &tCurr,
                          std::vector<VertexDesc> *factors) {
    if (!times_.empty()) {
      const auto &tPrev = times_.back();
      for (const auto &body: bodies_) {
        if (!body->isStatic()) {
          const PoseNoise2 pn =
            PoseNoise2::make(body->getFakeOdomRotationNoise(),
                             body->getFakeOdomTranslationNoise());
          const PoseWithNoise pwn(Transform::Identity(), pn, true);
          factors->push_back(OdometryProcessor::add_body_pose_delta(
                               graph_.get(), tPrev, tCurr, body, pwn));
        }
      }
    }
  }

  
  void TagSlam2::setupOdom(const std::vector<OdometryConstPtr> &odomMsgs) {
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
      odomProcessors_.push_back(OdometryProcessor(nh_, graph_, bpt));
    }
  }

  void
  TagSlam2::processOdom(const std::vector<OdometryConstPtr> &odomMsgs,
                        std::vector<VertexDesc> *factors) {
    if (odomProcessors_.empty()) {
      setupOdom(odomMsgs);
    }
    // from odom child frame id, deduce bodies
    for (const auto odomIdx: irange(0ul, odomMsgs.size())) {
      const auto &msg = odomMsgs[odomIdx];
      odomProcessors_[odomIdx].process(msg, factors);
      //graph_.test();
    }
  }

  Tag2ConstPtr TagSlam2::findTag(int tagId) {
    TagMap::iterator it = tagMap_.find(tagId);
    Tag2Ptr p;
    if (it == tagMap_.end()) {
      if (!defaultBody_) {
        ROS_WARN_STREAM("no default body, ignoring tag: " << tagId);
        return (p);
      } else {
        if (defaultBody_->ignoreTag(tagId)) {
          return (p);
        }
        p = Tag2::make(tagId, 6 /*num bits = tag family */,
                       defaultBody_->getDefaultTagSize(),
                       PoseWithNoise() /* invalid pose */, defaultBody_);
        defaultBody_->addTag(p);
        ROS_INFO_STREAM("new tag " << tagId << " attached to " <<
                        defaultBody_->getName());
        graph_utils::add_tag(graph_.get(), *p);
        auto iit = tagMap_.insert(TagMap::value_type(tagId, p));
        it = iit.first;
      }
    }
    return (it->second);
  }
  
  void TagSlam2::writeCameraPoses(const string &fname) const {
    std::ofstream f(fname);
    for (const auto &cam : cameras_) {
      f << cam->getName() << ":" << std::endl;
      PoseWithNoise pwn = graph_utils::get_optimized_camera_pose(*graph_,*cam);
      if (pwn.isValid()) {
        yaml_utils::write_pose_with_covariance(f, "  ", pwn.getPose(),
                                               pwn.getNoise());
      }
    }
  }

  void TagSlam2::writePoses(const string &fname) const {
    std::ofstream f(fname);
    const ros::Time t = times_.empty() ? ros::Time(0):*(times_.rbegin());
    f << "bodies:" << std::endl;
    const std::string idn = "       ";
    for (const auto &body: bodies_) {
      Transform bodyTF;
      const ros::Time tbody = body->isStatic() ? ros::Time(0) : t;
      f << " - " << body->getName() << ":" << std::endl;
      if (graph_utils::get_optimized_pose(*graph_, tbody, *body, &bodyTF)) {
        f << "    pose:" << std::endl;
        yaml_utils::write_pose(f, "       ",
                               bodyTF, PoseNoise2::make(0,0), true);
      }
      for (const auto &tag: body->getTags()) {
        Transform tagTF;
        if (graph_utils::get_optimized_pose(*graph_, *tag, &tagTF)) {
          f << idn << "- id: "   << tag->getId() << std::endl;
          f << idn << "  size: " << tag->getSize() << std::endl;
          f << idn << "  pose:" << std::endl;
          const auto &pwn = tag->getPoseWithNoise();
          yaml_utils::write_pose(f, idn + "    ", tagTF, pwn.getNoise(), true);
        }
      }
    }
  }

  void TagSlam2::writeTimeDiagnostics(const string &fname) const {
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

  void TagSlam2::writeErrorMap(const string &fname) const {
    std::ofstream f(fname);
    const auto errMap = graph_->getErrorMap();
    for (const auto &v: errMap) {
      const auto &vp = graph_->getVertex(v.second);
      f << FMT(8,3) << v.first << " ";
      write_time(f, vp->getTime());
      f << " " <<  *vp << std::endl;
    }
  }

  void TagSlam2::writeTagDiagnostics(const string &fname) const {
    std::ofstream f(fname);
    const std::string idn = "       ";
    for (const auto &body: bodies_) {
      for (const auto &tag: body->getTags()) {
        Transform tagTF;
        if (graph_utils::get_optimized_pose(*graph_, *tag, &tagTF)) {
          const auto &pwn = tag->getPoseWithNoise();
          if (pwn.isValid()) {
            const Transform poseDiff = tagTF * pwn.getPose().inverse();
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
  
  std::vector<Tag2ConstPtr>
  TagSlam2::findTags(const std::vector<Apriltag> &ta) {
    std::vector<Tag2ConstPtr> tpv;
    for (const auto &tag: ta) {
      Tag2ConstPtr tagPtr = findTag(tag.id);
      if (tagPtr) {
        tpv.push_back(tagPtr);
      } else {
        ROS_INFO_STREAM("ignoring tag as requested in config: " << tag.id);
      }
    }
    return (tpv);
  }

  void TagSlam2::processTags(const std::vector<TagArrayConstPtr> &tagMsgs,
                             std::vector<VertexDesc> *factors) {
    if (tagMsgs.size() != cameras_.size()) {
      ROS_ERROR_STREAM("tag msgs size mismatch!");
      return;
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
        Tag2ConstPtr tagPtr = findTag(tag.id);
        if (tagPtr) {
          const geometry_msgs::Point *corners = &(tag.corners[0]);
          TagProjectionFactorPtr fp(
            new factor::TagProjection(t, cam, tagPtr, corners, pixelNoise_,
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

  void TagSlam2::remapAndSquash(std::vector<TagArrayConstPtr> *remapped,
                                const std::vector<TagArrayConstPtr> &orig) {
    //
    // Sometimes there are tags with duplicate ids in the data set.
    // In this case, remap the tag ids of the detected tags dependent
    // on time stamp, to something else so they become unique.
    for (const auto &o: orig) {
      const ros::Time t = o->header.stamp;
      TagArrayPtr p(new TagArray());
      p->header = o->header;
      const auto sq = squash_.find(t);
      ROS_INFO_STREAM("time match: " << t << " " << (sq != squash_.end()));
      for (const auto &tag: o->apriltags) {
        if (sq != squash_.end() && sq->second.count(tag.id) != 0) {
          ROS_INFO_STREAM("squashed tag: " << tag.id);
        } else{
          p->apriltags.push_back(tag);
        }
      }
      for (auto &tag: p->apriltags) {
        auto it = tagRemap_.find(tag.id);
        if (it != tagRemap_.end()) {
          for (const ReMap &r: it->second) {
            if (t >= r.startTime && t <= r.endTime) {
              tag.id = r.remappedId;
            }
          }
        }
      }
      remapped->push_back(p);
    }
  }

  void TagSlam2::readRemap(XmlRpc::XmlRpcValue config) {
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

  static ros::Time parse_time(XmlRpc::XmlRpcValue v) {
    const std::string s = static_cast<std::string>(v);
    size_t pos = s.find(".", 0);
    if (pos == std::string::npos) {
      BOMB_OUT("bad ros time value: " << s);
    }
    const std::string nsec = s.substr(pos + 1, std::string::npos);
    const std::string sec  = s.substr(0, pos);
    if (nsec.size() != 9) {
      BOMB_OUT("ros nsec length is not 9 but: " << nsec.size());
    }
    ros::Time t(std::stoi(sec), std::stoi(nsec));
    return (t);
  }
 
  void TagSlam2::readSquash(XmlRpc::XmlRpcValue config) {
    if (!config.hasMember("squash")) {
      return;
    }
    XmlRpc::XmlRpcValue squash = config["squash"];
    if (squash.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (const auto i: irange(0, squash.size())) {
        if (squash[i].getType() !=
            XmlRpc::XmlRpcValue::TypeStruct) continue;
        ros::Time t(0);
        std::set<int> tags;
        for (XmlRpc::XmlRpcValue::iterator it = squash[i].begin();
             it != squash[i].end(); ++it) {
          if (it->first == "time") {
            t = parse_time(it->second);
          }
          if (it->first == "tags") {
            auto vtags = it->second;
            if (vtags.getType() == XmlRpc::XmlRpcValue::TypeArray) {
              for (const auto tag: irange(0, vtags.size())) {
                tags.insert(static_cast<int>(vtags[tag]));
              }
            }
          }
        }
        if (t != ros::Time(0) && !tags.empty()) {
          ROS_INFO_STREAM("squashing " << tags.size() << " tags for " << t);
          squash_[t] = tags;
        }
      }
    }
  }
  
  void
  TagSlam2::writeTagCorners(const ros::Time &t, int camIdx,
                            const Tag2ConstPtr &tag,
                            const geometry_msgs::Point *img_corners) {
    for (const auto i: irange(0, 4)) {
      tagCornerFile_ << t << " " << tag->getId() << " " << camIdx << " "
                     << i << " " << img_corners[i].x << " "
                     << img_corners[i].y << std::endl;
    }
  }


}  // end of namespace

