/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_slam2.h"
#include "tagslam/geometry.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/body_defaults.h"
#include "tagslam/body.h"
#include "tagslam/odometry_processor.h"
#include "tagslam/yaml_utils.h"

#include <cv_bridge/cv_bridge.h>

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
  
  static double find_size_of_tag(const geometry_msgs::Point *imgCorn) {
    Eigen::Matrix<double, 4, 2> x;
    x << imgCorn[0].x, imgCorn[0].y,
      imgCorn[1].x, imgCorn[1].y,
      imgCorn[2].x, imgCorn[2].y,
      imgCorn[3].x, imgCorn[3].y;
    // shoelace formula
    const double A = fabs(x(0,0)*x(1,1) + x(1,0)*x(2,1) + x(2,0)*x(3,1) + x(3,0)*x(0,1)
                          -x(1,0)*x(0,1) - x(2,0)*x(1,1) - x(3,0)*x(2,1) - x(0,0)*x(3,1));
    return (A);
  }
 
  TagSlam2::TagSlam2(const ros::NodeHandle &nh) : nh_(nh) {
  }


  void TagSlam2::sleep(double dt) const {
    ros::WallTime tw0 = ros::WallTime::now();
    for (ros::WallTime t = ros::WallTime::now(); t - tw0 < ros::WallDuration(dt);
         t = ros::WallTime::now()) {
      ros::spinOnce();
      ros::WallTime::sleepUntil(t + (t-tw0));
    }
  }

  bool TagSlam2::initialize() {
    nh_.param<std::string>("outbag", outBagName_, "out.bag");
    outBag_.open(outBagName_, rosbag::bagmode::Write);
    cameras_ = Camera2::parse_cameras("cameras", nh_);
    bool optFullGraph;
    nh_.param<bool>("optimize_full_graph", optFullGraph, false);
    nh_.param<double>("playback_rate", playbackRate_, 5.0);
    double pixelNoise, maxSubgraphError, angleLimit;
    nh_.param<double>("pixel_noise", pixelNoise, 1.0);
    graphManager_.setPixelNoise(pixelNoise);
    nh_.param<double>("minimum_viewing_angle", angleLimit, 20);
    graphManager_.setAngleLimit(angleLimit);
    nh_.param<double>("max_subgraph_error", maxSubgraphError, 50.0);
    graphManager_.setMaxSubgraphError(maxSubgraphError);
    ROS_INFO_STREAM("found " << cameras_.size() << " cameras");
    graphManager_.setOptimizeFullGraph(optFullGraph);
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
        graphManager_.addPose(ros::Time(0), Graph::cam_name(cam->getName()),
                              Transform::Identity(), false, true/*camPose*/);
      } else {
        camHasKnownPose = true;
        ROS_INFO_STREAM("camera " << cam->getName() << " has known pose!");
        graphManager_.addPoseWithPrior(ros::Time(0),
                                Graph::cam_name(cam->getName()), pwn);
      }
    }
    if (!camHasKnownPose) {
      ROS_ERROR("at least one camera must have known pose!");
      return (false);
    }
    //
    graphManager_.optimize(0);
    nh_.param<string>("fixed_frame_id", fixedFrame_, "map");
    nh_.param<int>("max_number_of_frames", maxFrameNum_, 1000000);
    nh_.param<bool>("write_debug_images", writeDebugImages_, false);
    nh_.param<bool>("has_compressed_images", hasCompressedImages_, false);
    
    string bagFile;
    nh_.param<string>("bag_file", bagFile, "");
    clockPub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", QSZ);
    service_ = nh_.advertiseService("replay", &TagSlam2::replay, this);

    sleep(1.0);
    playFromBag(bagFile);
    //graphManager_.plotDebug(ros::Time(0), "final");
    outBag_.close();
    std::string camPoseFile;
    nh_.param<string>("camera_poses_out_file",
                      camPoseFile, "camera_poses.yaml");
    writeCameraPoses(camPoseFile);
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
      graphManager_.addBody(*body);
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
    res.message = "replayed " + std::to_string(times_.size());
    res.success = true;
    ROS_INFO_STREAM("finished replaying " << times_.size());
    outBag_.close();
    return (true);
  }

  void TagSlam2::publishTransforms(const ros::Time &t) {
    tf::tfMessage tfMsg;
    geometry_msgs::TransformStamped tfm;
    for (const auto &body: bodies_) {
      Transform bodyTF;
      const string &bodyFrameId = body->getFrameId();
      const ros::Time ts = body->isStatic() ? ros::Time(0) : t;
      if (graphManager_.getPose(ts, Graph::body_name(body->getName()), &bodyTF)) {
        //ROS_INFO_STREAM("publishing pose for body " << body->getName() << std::endl << bodyTF);
        tf::StampedTransform btf = tf::StampedTransform(to_tftf(bodyTF), t, fixedFrame_, bodyFrameId);
        tf::transformStampedTFToMsg(btf, tfm);
        tfMsg.transforms.push_back(tfm);
        tfBroadcaster_.sendTransform(btf);
        for (const auto &tag: body->getTags()) {
          Transform tagTF;
          if (graphManager_.getPose(ros::Time(0), Graph::tag_name(tag->getId()), &tagTF)) {
            const std::string frameId = "tag_" + std::to_string(tag->getId());
            auto ttf = tf::StampedTransform(to_tftf(tagTF), t, bodyFrameId, frameId);
            tfBroadcaster_.sendTransform(ttf);
            //ROS_INFO_STREAM("publishing pose for tag " << tag->getId() << std::endl << tagTF);
            tf::transformStampedTFToMsg(ttf, tfm);
            tfMsg.transforms.push_back(tfm);
          } else {
            //ROS_INFO_STREAM(t<< " no pose for tag " << tag->getId());
          }
        }
      } else {
        ROS_INFO_STREAM(t<< " no pose for " << body->getName());
      }
    }
    for (const auto &cam: cameras_) {
      Transform camTF;
      if (graphManager_.getPose(ros::Time(0), Graph::cam_name(cam->getName()), &camTF)) {
        const string &rigFrameId = cam->getRig()->getFrameId();
        auto ctf = tf::StampedTransform(to_tftf(camTF), t, rigFrameId, cam->getFrameId());
        tfBroadcaster_.sendTransform(ctf);
        tf::transformStampedTFToMsg(ctf, tfm);
        tfMsg.transforms.push_back(tfm);
        //ROS_DEBUG_STREAM("published transform for cam: " << cam->getName());
      }
    }
    outBag_.write<tf::tfMessage>("/tf", t, tfMsg);
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
    graphManager_.reoptimize();
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
      if (graphManager_.getPose(t, Graph::body_name(body->getName()), &pose)) {
        //ROS_INFO_STREAM("publishing odom for body " << body->getName() << std::endl << pose);
        auto msg = make_odom(t, fixedFrame_, body->getOdomFrameId(), pose);
        odomPub_[body_idx].publish(msg);
        outBag_.write<nav_msgs::Odometry>("odom/body_" + body->getName(), t, msg);
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
      graphManager_.addPose(t, Graph::body_name(body->getName()),
                     Transform::Identity(), false);
    }
    std::vector<VertexDesc> factors;
    profiler_.reset();
#define USE_ODOM
#ifdef USE_ODOM
    if (odommsgs.size() != 0) {
      processOdom(odommsgs, &factors);
    }
    profiler_.record("processOdom");
#endif
    processTags(tagmsgs, &factors);
    profiler_.record("processTags");
    graphManager_.processNewFactors(t, factors);
    profiler_.record("processNewFactors");
    //graphManager_.plotDebug(tagMsgs[0]->header.stamp, "factors");
    times_.push_back(t);
    publishAll(t);
    profiler_.record("publishAll");
    std::cout << profiler_ << std::endl;
    std::cout.flush();
    frameNum_++;
  }

  void TagSlam2::publishAll(const ros::Time &t) {
    publishBodyOdom(t);
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = t;
    clockPub_.publish(clockMsg);
    publishTransforms(t);
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
      odomProcessors_.push_back(OdometryProcessor(nh_, &graphManager_, bpt));
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
          ROS_INFO_STREAM("ignoring tag as requested in config: " << tagId);
          return (p);
        }
        p = Tag2::make(tagId, 6 /*num bits = tag family */,
                       defaultBody_->getDefaultTagSize(),
                       PoseWithNoise() /* invalid pose */, defaultBody_);
        defaultBody_->addTag(p);
        ROS_INFO_STREAM("new tag " << tagId << " attached to " <<
                        defaultBody_->getName());
        graphManager_.addTag(*p);
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
      PoseWithNoise pwn = graphManager_.getCameraPoseWithNoise(cam);
      if (pwn.isValid()) {
        yaml_utils::write_pose_with_covariance(f, "  ", pwn.getPose(),
                                               pwn.getNoise());
      }
    }
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
      if (!tagMsgs[i]->apriltags.empty()) {
        // insert time-dependent camera pose
        graphManager_.addPose(t, Graph::cam_name(cam->getName()),
                              Transform::Identity(), false, true/*camPose*/);
        // and tie it to the time-independent camera pose
        // with a relative prior
        const PoseWithNoise pn(Transform::Identity(), PoseNoise2::make(0.01, 0.01), true);
        string  name = Graph::cam_name(cam->getName());
        RelativePosePriorFactorPtr fac(new factor::RelativePosePrior(t, ros::Time(0), pn, name));
        VertexDesc v = graphManager_.addRelativePosePrior(fac);
        sortedFactors.insert(MMap::value_type(1e10, v));
      }
      for (const auto &tag: tagMsgs[i]->apriltags) {
        Tag2ConstPtr tagPtr = findTag(tag.id);
        if (tagPtr) {
          const geometry_msgs::Point *img_corners = &(tag.corners[0]);
          auto fac = graphManager_.addProjectionFactor(t, tagPtr, cam,
                                                       img_corners);
          double sz = find_size_of_tag(img_corners);
          ROS_DEBUG_STREAM("cam " << cam->getName() << " obs tag " << tag.id << " with size: " << sz);
          sortedFactors.insert(MMap::value_type(sz, fac));
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

}  // end of namespace

