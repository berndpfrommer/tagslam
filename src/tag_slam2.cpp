/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_slam2.h"
#include "tagslam/geometry.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/body_defaults.h"
#include "tagslam/body.h"
#include "tagslam/odometry_processor.h"

#include <cv_bridge/cv_bridge.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <boost/range/irange.hpp>

const int QSZ = 1000;
namespace tagslam {
  using Odometry = nav_msgs::Odometry;
  using OdometryConstPtr = nav_msgs::OdometryConstPtr;
  using Image = sensor_msgs::Image;
  using ImageConstPtr = sensor_msgs::ImageConstPtr;
  using CompressedImage = sensor_msgs::CompressedImage;
  using CompressedImageConstPtr = sensor_msgs::CompressedImageConstPtr;
  using boost::irange;

  static tf::Transform to_tftf(const Transform &tf) {
    tf::Transform ttf;
    tf::transformEigenToTF(tf, ttf);
    return (ttf);
  }
  
  TagSlam2::TagSlam2(const ros::NodeHandle &nh) : nh_(nh) {
  }

  /*
  void TagSlam2::addGraphToOptimizer(BoostGraph *graph) {
    BoostGraph &g = *graph;
    ValuesPredicate<BoostGraph> valueFilter(g, true);
    ValuesPredicate<BoostGraph> factorFilter(g, false);
    typedef boost::filtered_graph<
      BoostGraph, boost::keep_all, ValuesPredicate<BoostGraph>> FilteredGraph;
    FilteredGraph values(g, boost::keep_all(), valueFilter);
    FilteredGraph factors(g, boost::keep_all(), factorFilter);

    typedef FilteredGraph::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vp;
    // first add values (must come first!)
    for (vp = vertices(values); vp.first != vp.second; ++vp.first) {
      g[*vp.first].vertex->addToOptimizer(this, *vp.first, &g);
    }
    // then add factors
    for (vp = vertices(factors); vp.first != vp.second; ++vp.first) {
      g[*vp.first].vertex->addToOptimizer(this, *vp.first, &g);
    }
  }
  */


  void TagSlam2::sleep(double dt) const {
    ros::Rate r(10); // 10 hz
    ros::WallTime tw0 = ros::WallTime::now();
    while (ros::WallTime::now() - tw0 < ros::WallDuration(0.2)) {
      ros::spinOnce();
      ros::WallTime::sleepUntil(ros::WallTime::now() + ros::WallDuration(0.01));
    }
    ROS_INFO_STREAM("done sleeping!");
  }

  bool TagSlam2::initialize() {
    cameras_ = Camera2::parse_cameras("cameras", nh_);
    bool optFullGraph;
    nh_.param<bool>("optimize_full_graph", optFullGraph, false);
    ROS_INFO_STREAM("found " << cameras_.size() << " cameras");
    graph_.setOptimizer(&optimizer_);
    graph_.setOptimizeFullGraph(optFullGraph);
    readBodies();
    //
    //optimizer_.optimizeFullGraph();
    optimizer_.optimize();
    nh_.param<string>("fixed_frame_id", fixedFrame_, "map");
    nh_.param<int>("max_number_of_frames", maxFrameNum_, 1000000);
    nh_.param<bool>("write_debug_images", writeDebugImages_, false);
    nh_.param<bool>("has_compressed_images", hasCompressedImages_, false);
    
    string bagFile;
    nh_.param<string>("bag_file", bagFile, "");
    clockPub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", QSZ);
    sleep(1.0);
    playFromBag(bagFile);
    return (true);
  }

  void TagSlam2::readBodies() {
    XmlRpc::XmlRpcValue config;
    nh_.getParam("tagslam_config", config);

    // read body defaults first in case
    // bodies do not provide all parameters
    BodyDefaults::parse(config);

    // now read bodies
    bodies_ = Body::parse_bodies(config);
    for (const auto &body: bodies_) {
      graph_.addBody(*body);
      if (!body->isStatic()) {
        nonstaticBodies_.push_back(body);
        odomPub_.push_back(
          nh_.advertise<nav_msgs::Odometry>("odom/body_"+body->getName(), QSZ));
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

  std::vector<std::vector<std::string>>
  TagSlam2::makeTopics(rosbag::Bag *bag) const {
    std::vector<std::vector<string>> topics(3);
    for (const auto &body: bodies_) {
      if (!body->getOdomTopic().empty()) {
        topics[2].push_back(body->getOdomTopic());
      }
    }
    for (const auto &cam: cameras_) {
      if (cam->getTagTopic().empty()) {
        ROS_ERROR_STREAM("camera " << cam->getName() << " no tag topic!");
        throw (std::runtime_error("camera has no tag topic!"));
      }
      topics[0].push_back(cam->getTagTopic());
      if (writeDebugImages_) {
        if (cam->getImageTopic().empty()) {
          ROS_ERROR_STREAM("camera " << cam->getName() << " no image topic!");
          throw (std::runtime_error("camera has no image topic!"));
        }
        topics[1].push_back(cam->getImageTopic());
      }
    }
    // test if in bag
    for (const auto &topic: topics) {
      for (const auto &t: topic) {
        rosbag::View v(*bag, rosbag::TopicQuery({t}));
        if (v.begin() == v.end()) {
          ROS_WARN_STREAM("cannot find topic: " << t <<
                          " in bag, sync will fail!");
        } else {
          ROS_INFO_STREAM("playing topic: " << t);
        }
      }
    }
    return (topics);
  }

  void TagSlam2::publishTransforms(const ros::Time &t) {
    for (const auto &body: bodies_) {
      Transform bodyTF;
      const string &bodyFrameId = body->getFrameId();
      if (graph_.getPose(t, "body:" + body->getName(), &bodyTF)) {
        tfBroadcaster_.sendTransform(
          tf::StampedTransform(to_tftf(bodyTF), t, fixedFrame_, bodyFrameId));
        for (const auto &tag: body->getTags()) {
          Transform tagTF;
          if (graph_.getPose(t, "tag:" + tag->getId(), &tagTF)) {
            const std::string frameId = "tag_" + std::to_string(tag->getId());
            tfBroadcaster_.sendTransform(
              tf::StampedTransform(to_tftf(tagTF), t, bodyFrameId, frameId));
          }
        }
      }
    }
  }

  void TagSlam2::playFromBag(const string &fname) {
    rosbag::Bag bag;
    ROS_INFO_STREAM("reading from bag: " << fname);
    bag.open(fname, rosbag::bagmode::Read);
    std::vector<std::vector<string>> topics = makeTopics(&bag);
    std::vector<string> flatTopics;
    for (const auto &v: topics) {
      flatTopics.insert(flatTopics.begin(), v.begin(), v.end());
    }
    double deltaStartTime{0};
    nh_.param<double>("bag_start_time", deltaStartTime, 0.0);
    rosbag::View dummyView(bag);
    const ros::Time startTime =
      dummyView.getBeginTime() + ros::Duration(deltaStartTime);
    publishTransforms(startTime);

    rosbag::View view(bag, rosbag::TopicQuery(flatTopics), startTime);
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

  static nav_msgs::Odometry  make_odom(const ros::Time &t,
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
      if (graph_.getPose(t, "body:" + body->getName(), &pose)) {
        ROS_INFO_STREAM("publishing pose: " << body->getName());
        odomPub_[body_idx].publish(
          make_odom(t, fixedFrame_, body->getOdomFrameId(), pose));
      }
    }
  }

  void TagSlam2::processTagsAndOdom(
    const std::vector<TagArrayConstPtr> &tagmsgs,
    const std::vector<OdometryConstPtr> &odommsgs) {
    if (tagmsgs.empty() && odommsgs.empty()) {
      ROS_ERROR_STREAM("neither tags nor odom!");
      return;
    }

    const ros::Time t = tagmsgs.empty() ?
      odommsgs[0]->header.stamp : tagmsgs[0]->header.stamp;

    for (const auto &body: nonstaticBodies_) {
      graph_.addPose(t, "body:"+body->getName(), Transform::Identity(), false);
    }
#define USE_ODOM
#ifdef USE_ODOM
    if (odommsgs.size() == 0) {
      return;
    }
    processOdom(odommsgs);
    graph_.optimize();
    //if (!msgvec3.empty()) {
    //graph_.plotDebug(msgvec3[0]->header.stamp, "odom");
    //}
#endif    
    processTags(tagmsgs);
    publishBodyOdom(t);
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = t;
    clockPub_.publish(clockMsg);
    publishTransforms(t);

    frameNum_++;
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
        ROS_ERROR_STREAM("no body found for odom frame id: " << frameId);
        throw (std::runtime_error("no body found for frame id!"));
      }
      if (bodySet.count(bpt) != 0) {
        ROS_WARN_STREAM("multiple bodies with frame id: " << frameId);
        ROS_WARN_STREAM("This will screw up the odom!");
      }
      ROS_INFO_STREAM("getting odom from " << bpt->getName() << " " << frameId);
      odomProcessors_.push_back(OdometryProcessor(&graph_, bpt));
    }
  }

  void TagSlam2::processOdom(const std::vector<OdometryConstPtr> &odomMsgs) {
    if (odomProcessors_.empty()) {
      setupOdom(odomMsgs);
    }
    // from odom child frame id, deduce bodies
    for (const auto odomIdx: irange(0ul, odomMsgs.size())) {
      const auto &msg = odomMsgs[odomIdx];
      odomProcessors_[odomIdx].process(msg);
      //graph_.test();
    }
  }

  void TagSlam2::processTags(const std::vector<TagArrayConstPtr> &tagMsgs) {
    if (tagMsgs.size() != cameras_.size()) {
      ROS_ERROR_STREAM("tag msgs size mismatch!");
      return;
    }
    graph_.addTagMeasurements(bodies_, tagMsgs, cameras_);
  }

}  // end of namespace

