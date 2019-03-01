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
#include <geometry_msgs/Point.h>
#include <XmlRpcException.h>

#include <boost/range/irange.hpp>

#include <sstream>

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
    bool camHasKnownPose(false);
    for (auto &cam: cameras_) {
      for (const auto &body: bodies_) {
        if (body->getName() == cam->getRigName()) {
          cam->setRig(body);
        }
      }
      if (!cam->getRig()) {
        ROS_ERROR_STREAM("rig body not found: " << cam->getRigName());
        throw (std::runtime_error("rig body not found!"));
      }
      PoseWithNoise pwn = PoseWithNoise::parse(cam->getName(), nh_);
      if (!pwn.isValid()) {
        ROS_INFO_STREAM("camera " << cam->getName() << " has no pose!");
        graph_.addPose(ros::Time(0), Graph::cam_name(cam->getName()),
                       Transform::Identity(), false);
      } else {
        camHasKnownPose = true;
        ROS_INFO_STREAM("camera " << cam->getName() << " has known pose!");
        graph_.addPoseWithPrior(ros::Time(0),
                                Graph::cam_name(cam->getName()), pwn);
      }
    }
    if (!camHasKnownPose) {
      ROS_ERROR("at least one camera must have known pose!");
      return (false);
    }
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
    graph_.plotDebug(ros::Time(0), "final");
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
      // add associated tags as vertices
      for (const auto &tag: body->getTags()) {
        tagMap_.insert(TagMap::value_type(tag->getId(), tag));
      }

      if (!body->isStatic()) {
        nonstaticBodies_.push_back(body);
        odomPub_.push_back(
          nh_.advertise<nav_msgs::Odometry>("odom/body_"+body->getName(), QSZ));
      }
    }
    nh_.getParam("tagslam_config", config);
    try {
      const string defbody = config["default_body"];
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
      if (graph_.getPose(t, Graph::body_name(body->getName()), &bodyTF)) {
        tfBroadcaster_.sendTransform(
          tf::StampedTransform(to_tftf(bodyTF), t, fixedFrame_, bodyFrameId));
        for (const auto &tag: body->getTags()) {
          Transform tagTF;
          if (graph_.getPose(t, Graph::tag_name(tag->getId()), &tagTF)) {
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
      if (graph_.getPose(t, Graph::body_name(body->getName()), &pose)) {
        //ROS_INFO_STREAM("publishing pose: " << body->getName());
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

    // add unknown poses for all non-static bodies
    for (const auto &body: nonstaticBodies_) {
      graph_.addPose(t, Graph::body_name(body->getName()),
                     Transform::Identity(), false);
    }
//#define USE_ODOM
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

  Tag2ConstPtr TagSlam2::findTag(int tagId) {
    auto it = tagMap_.find(tagId);
    Tag2Ptr p;
    if (it == tagMap_.end()) {
      if (!defaultBody_) {
        ROS_WARN_STREAM("no default body, ignoring tag: " << tagId);
        return (p);
      } else {
        p = Tag2::make(tagId, 6 /*num bits = tag family */,
                       defaultBody_->getDefaultTagSize(),
                       PoseWithNoise() /* invalid pose */, defaultBody_);
        defaultBody_->addTag(p);
        ROS_INFO_STREAM("new tag " << tagId << " attached to " <<
                        defaultBody_->getName());
        graph_.addTag(*p);
        auto iit = tagMap_.insert(TagMap::value_type(tagId, p));
        it = iit.first;
      }
    }
    return (it->second);
  }

  void TagSlam2::processTags(const std::vector<TagArrayConstPtr> &tagMsgs) {
    if (tagMsgs.size() != cameras_.size()) {
      ROS_ERROR_STREAM("tag msgs size mismatch!");
      return;
    }
    std::vector<BoostGraphVertex> factors;
    for (const auto i: irange(0ul, cameras_.size())) {
      const ros::Time &t = tagMsgs[i]->header.stamp;
      for (const auto &tag: tagMsgs[i]->apriltags) {
        Tag2ConstPtr tagPtr = findTag(tag.id);
        if (tagPtr) {
          const geometry_msgs::Point *img_corners = &(tag.corners[0]);
          auto fac = graph_.addProjectionFactor(t, tagPtr, cameras_[i],
                                                img_corners);
          factors.push_back(fac);
        }
      }
      std::stringstream ss;
      for (const auto &tag: tagMsgs[i]->apriltags) {
        ss << " " << tag.id;
      }
      ROS_INFO_STREAM("frame " << frameNum_ << " " << cameras_[i]->getName()
                      << " sees tags: " << ss.str());
    }
  }

}  // end of namespace

