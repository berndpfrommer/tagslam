/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_slam.h"
#include "tagslam/pose_estimate.h"
#include "tagslam/tag.h"
#include "tagslam/yaml_utils.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>

namespace tagslam {
  using boost::irange;
  struct PoseError {
    PoseError(double r = 0, double t = 0) : rot(r), trans(t) {
    }
    double rot{0};
    double trans{0};
  };

  TagSlam::TagSlam(const ros::NodeHandle& pnh) :  nh_(pnh) {
  }

  TagSlam::~TagSlam() {
  }

  bool TagSlam::initialize() {
    cameras_ = Camera::parse_cameras(nh_);
    if (cameras_.empty()) {
      ROS_ERROR("no cameras found!");
      return (false);
    }
    if (!subscribe()) {
      return (false);
    }
    for (const auto cam_idx : irange(0ul, cameras_.size())) {
      const auto &ci = cameras_[cam_idx].intrinsics;
      tagGraph_.addCamera(cam_idx, ci.intrinsics,
                          ci.distortion_model,
                          ci.distortion_coeffs);
      initialPoseGraph_.addCamera(cam_idx, ci.intrinsics,
                                  ci.distortion_model,
                                  ci.distortion_coeffs);
    }
    XmlRpc::XmlRpcValue static_objects;
    nh_.getParam("tag_poses/static_objects", static_objects);
    nh_.param<double>("default_tag_size", defaultTagSize_, 0.04);
    nh_.param<bool>("discover_tags", discoverTags_, true);
    nh_.param<std::string>("tag_poses_out_file", tagPosesOutFile_,
                           "static_poses_out.yaml");
    if (static_objects.getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
      ROS_ERROR("cannot find static_objects in yaml file!");
      return (false);
    }
    for (const auto i: irange(0, static_objects.size())) {
      if (static_objects[i].getType() !=
          XmlRpc::XmlRpcValue::TypeStruct) continue;
      //
      for (XmlRpc::XmlRpcValue::iterator it = static_objects[i].begin();
           it != static_objects[i].end(); ++it) {
        if (it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
          ROS_ERROR_STREAM("object " << it->first << " must be of type struct!");
          continue;
        }
        parseStaticObject(it->first, it->second);
      }
    }
    if (staticObjects_.empty()) {
      staticObjects_.push_back(StaticObject("world", gtsam::Pose3(),
                                            utils::make_pose_noise(1e-4, 1e-4)));
    }
    std::string bagFile;
    nh_.param<std::string>("bag_file", bagFile, "");
    if (!bagFile.empty()) {
      playFromBag(bagFile);
      ros::shutdown();
    }
    return (true);
  }

  bool TagSlam::subscribe() {
    if (cameras_.size() == 1) {
      singleCamSub_ = nh_.subscribe(cameras_[0].tagtopic, 1,
                                    &TagSlam::callback1, this);
    } else {
      for (const auto &cam : cameras_) {
        sub_.push_back(std::shared_ptr<TagSubscriber>(
                         new TagSubscriber(nh_, cam.tagtopic, 1)));
      }
      switch (cameras_.size()) {
      case 2:
        approxSync2_.reset(new TimeSync2(SyncPolicy2(60/*q size*/),
                                         *(sub_[0]), *(sub_[1])));
        approxSync2_->registerCallback(&TagSlam::callback2, this);
        break;
      case 3:
        approxSync3_.reset(new TimeSync3(SyncPolicy3(60/*q size*/),
                                         *(sub_[0]), *(sub_[1]), *(sub_[2])));
        approxSync3_->registerCallback(&TagSlam::callback3, this);
        break;
      default:
        ROS_ERROR_STREAM("number of cameras too large: " << cameras_.size());
        return (false);
        break;
      }
    }
    return (true);
  }

  int TagSlam::findTagType(double size) {
    // check if this tag is of unknown size. If yes,
    // declare it a new type.
    if (tagTypeMap_.count(size) == 0) {
      tagTypeMap_.insert(std::map<double,int>::value_type(size, tagTypeMap_.size()));
    }
    return (tagTypeMap_.find(size)->second);
  }

  void TagSlam::parseStaticObject(const std::string &name,
                                  XmlRpc::XmlRpcValue &staticObject) {
    gtsam::Pose3 pose;
    utils::PoseNoise noise;

    for (XmlRpc::XmlRpcValue::iterator it = staticObject.begin();
         it != staticObject.end(); ++it) {
      if (it->first == "pose" &&
          it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        yaml_utils::get_pose_and_noise(it->second, &pose, &noise);
        break;
      }
    }
    unsigned int objIdx  = staticObjects_.size();
    staticObjects_.push_back(StaticObject(name, pose, noise));

    for (XmlRpc::XmlRpcValue::iterator it = staticObject.begin();
         it != staticObject.end(); ++it) {
      if (it->first == "tags") {
        TagVec tags = Tag::parseTags(it->second);
        ROS_INFO_STREAM("adding static object: " << name <<
                        " with " << tags.size() << "tags");
        for (Tag &t: tags) {
          if (idToTag_.count(t.id) != 0) {
            ROS_ERROR_STREAM("duplicate tag id in input file: " << t.id);
            ros::shutdown();
          }
          t.type = findTagType(t.size);
          t.parentIdx = objIdx;
          t.initialPoseKnown = true;
          idToTag_.insert(IdToTagMap::value_type(t.id, t));
          staticObjects_.back().tagIds.insert(t.id);
          ROS_INFO_STREAM("adding to static object " <<
                          name << " tag with id: " << t.id);
        }
        tagGraph_.addTags(name, pose, noise, tags);
        return;
      }
    }
    ROS_WARN_STREAM("warning: ignoring object " << name << " without tags");
  }

  static ros::Time get_latest_time(const std::vector<TagArrayConstPtr> &msgvec) {
    ros::Time t(0);
    for (const auto &m: msgvec) {
      if (m->header.stamp > t) t = m->header.stamp;
    }
    return (t);
  }

//#define DEBUG_CAM_POSE_INIT


  void TagSlam::makeTagPoints(std::vector<gtsam::Point2> *imgPoints,
                              std::vector<gtsam::Point3> *worldPoints,
                              std::vector<cv::Point2d> *imgPointsCv,
                              std::vector<cv::Point3d> *worldPointsCv,
                              const TagVec &tags) const {
    std::vector<cv::Point2d> &ip = *imgPointsCv;
    std::vector<cv::Point3d> &wp = *worldPointsCv;
#ifdef DEBUG_CAM_POSE_INIT
    std::cout << "&&&&&&&&&&& tag points &&&&&&&&&&&&&" << std::endl;
#endif    
    for (const auto &tag: tags) {
      for (const auto i: irange(0, 4)) {
        gtsam::Point2 uv  = tag.corners[i];
        const gtsam::Pose3  &T_w_s =
          staticObjects_[tag.parentIdx].pose;
        gtsam::Point3 p =  T_w_s * tag.pose * tag.getObjectCorner(i);
        worldPoints->push_back(p);
        wp.push_back(cv::Point3d(p.x(), p.y(), p.z()));
        imgPoints->push_back(uv);
        ip.push_back(cv::Point2d(uv.x(), uv.y()));
#ifdef DEBUG_CAM_POSE_INIT
        std::cout << tag.id << " " << ip.back().x << " " << ip.back().y << " "
                  << wp.back().x << " " << wp.back().y << " " << wp.back().z
                  << std::endl;
#endif
      }
    }
  }

  PoseEstimate
  TagSlam::estimatePosePNP(int cam_idx,
                           const std::vector<cv::Point2d>&ip,
                           const std::vector<cv::Point3d>&wp,
                           const gtsam::Pose3 &prevPose,
                           bool hasValidPrevPose) const {
    PoseEstimate pe;
    pe.err = 1e10;
    if (!ip.empty()) {
      const auto   &ci  = cameras_[cam_idx].intrinsics;
      cv::Mat rvec, tvec;
      bool rc = utils::get_init_pose_pnp(wp, ip, ci.K,
                                         ci.distortion_model,
                                         ci.D, &rvec, &tvec);
      if (rc) {
        gtsam::Vector tvec_gtsam = (gtsam::Vector(3) <<
                                    tvec.at<double>(0),
                                    tvec.at<double>(1),
                                    tvec.at<double>(2)).finished();
        gtsam::Pose3 T_c_w(gtsam::Rot3::rodriguez(
                                   rvec.at<double>(0),
                                   rvec.at<double>(1),
                                   rvec.at<double>(2)), tvec_gtsam);
        pe.pose = T_c_w.inverse();
        pe.err = utils::reprojection_error(wp, ip, rvec, tvec, ci.K,
                                           ci.distortion_model, ci.D);
        pe.poseChange = PoseEstimate::pose_change(prevPose, pe.pose);
      }
    }
    return (pe);
  }
                              

  PoseEstimate
  TagSlam::estimateCameraPose(int cam_idx, const TagVec &tags) {
    std::vector<cv::Point2d>   ip;
    std::vector<cv::Point3d>   wp;
    std::vector<gtsam::Point2> imgPts;
    std::vector<gtsam::Point3> worldPts;
    makeTagPoints(&imgPts, &worldPts, &ip, &wp, tags);

    gtsam::Pose3 prevPose;
    bool hasPrevPose = tagGraph_.getCameraPose(cam_idx, &prevPose);
    //
    // first try estimating pose via PNP off of the known tags
    //
    PoseEstimate pe  = estimatePosePNP(cam_idx, ip, wp,
                                       prevPose, hasPrevPose);
    if (pe.err > 10.0) {
      // PNP failed, try with a local graph
      pe = initialPoseGraph_.estimateCameraPose(cam_idx, worldPts, imgPts,
                                                prevPose, hasPrevPose);
      if (pe.err > 10.0) {
        ROS_WARN_STREAM("cannot find pose of camera, err: " << pe.err);
        pe.err = 1e10;
      }
    }
    return (pe);
  }

  void TagSlam::updateTagPosesFromGraph(const TagVec &tags) {
    for (const auto &tag: tags) {
      const gtsam::Pose3 T_w_o = tagGraph_.getTagWorldPose(tag.id);
      Tag &newTag = idToTag_[tag.id];
      //std::cout << "initial guess: " << std::endl << newTag.pose << std::endl;
      //std::cout << "updated to: " << std::endl << T_w_o << std::endl;
      newTag.pose = T_w_o;
    }
  }

  Tag TagSlam::makeTag(int id, double size, const gtsam::Pose3 &pose,
                       const Tag::PoseNoise &noise,
                       const geometry_msgs::Point *corners,
                       int parentIdx) {
    Tag newTag(id, findTagType(size), size, pose, noise);
    newTag.parentIdx = 0; // default to "first static object"
    newTag.setCorners(corners);
    return (newTag);
  }


  void TagSlam::findKnownTags(const TagArrayConstPtr &observedTags,
                              TagVec *knownTags,
                              TagVec *unknownTags) {
    for (const auto &tag: observedTags->apriltags) {
      if (idToTag_.empty()) {
        // no pre-defined tags, make first visible tag
        // the center of world!
        Tag::PoseNoise tagNoise = utils::make_pose_noise(1e-4, 1e-4);
        gtsam::Pose3   tagPose;
        Tag newTag = makeTag(tag.id, defaultTagSize_, tagPose,
                             tagNoise, &tag.corners[0], 0);
        newTag.initialPoseKnown = true;
        knownTags->push_back(newTag);
        StaticObject &so = staticObjects_[0];
        so.tagIds.insert(tag.id);
        tagGraph_.addTags(so.name, so.pose, so.noise, *knownTags);
        return; // ignore all other tags for now
      }
      const auto it = idToTag_.find(tag.id);
      if (it != idToTag_.end()) {
        // tag is known, just update corners
        Tag newTag(it->second);
        newTag.setCorners(&tag.corners[0]);
        knownTags->push_back(newTag);
      } else {
        if (discoverTags_) {
          // unknown tag, will determine approximate pose later
          ROS_INFO_STREAM("found new tag with id " << tag.id);
          Tag::PoseNoise tagNoise = utils::make_pose_noise(30, 10);
          Tag newTag = makeTag(tag.id, defaultTagSize_, gtsam::Pose3(),
                               tagNoise, &tag.corners[0], 0);
          unknownTags->push_back(newTag);
        }
      }
    }
  }
        
  void TagSlam::process(const std::vector<TagArrayConstPtr> &msgvec) {
    for (const auto cam_idx: irange(0ul, msgvec.size())) {
      const auto tags = msgvec[cam_idx];
      TagVec knownTags, unknownTags;
      findKnownTags(tags, &knownTags, &unknownTags);
      gtsam::Pose3 cameraPose;
      double err;
      // estimate camera just from the known tags
      PoseEstimate pe = estimateCameraPose(cam_idx, knownTags);
      if (pe.err < 1e10) {
        // for any discovered unknown tags, determine their initial pose
        TagVec tagsWithPoses;
        findInitialTagPoses(&tagsWithPoses, unknownTags, cam_idx, pe.pose);
        tagGraph_.addTags("unknown", gtsam::Pose3(),
                          utils::make_pose_noise(0.001, 0.001), tagsWithPoses);
        // the unknown tags have now become known tags!
        for (const auto &tag : tagsWithPoses) {
          knownTags.push_back(tag);
          idToTag_[tag.id] = tag;
          staticObjects_[0].tagIds.insert(tag.id);
        }
        // this will also run the optimizer
        tagGraph_.observedTags(cam_idx, knownTags, frameNum_, pe.pose);
        // after the optimizer has run, update
        // the tag poses one more time
        updateTagPosesFromGraph(knownTags);
        ROS_INFO_STREAM(tags->header.stamp << " " << knownTags.size()
                        << " tags cam: " << cam_idx << " init err: " << pe.err
                        << " opt err: "  << tagGraph_.getError()
                        << " opt iter: " << tagGraph_.getIterations());
        std::cout << std::flush;
      }
    }
    ros::Time t = get_latest_time(msgvec);
    broadcastCameraPoses(t);
    broadcastTagPoses(t);
    writeTagPoses(tagPosesOutFile_);
    frameNum_++;
  }

  void TagSlam::findInitialTagPoses(TagVec *tagsWithPoses,
                                    const TagVec &tagsWithoutPoses,
                                    int cam_idx,
                                    const gtsam::Pose3 &T_w_c) {
    for (auto &tag: tagsWithoutPoses) {
      gtsam::Pose3 tagPose;
      if (estimateInitialTagPose(cam_idx, T_w_c, &tag.corners[0], &tagPose)) {
        Tag t(tag);
        t.pose = tagPose;
        tagsWithPoses->push_back(t);
      }
    }
  }

  static tf::Transform gtsam_pose_to_tf(const gtsam::Pose3 &p) {
    tf::Transform tf;
    const gtsam::Vector rpy = p.rotation().rpy();
    tf.setOrigin(tf::Vector3(p.x(), p.y(), p.z()));
    tf::Quaternion q;
    q.setRPY(rpy(0), rpy(1), rpy(2));
    tf.setRotation(q);
    return (tf);
  }


  void TagSlam::broadcastCameraPoses(const ros::Time &t) {
    std::vector<std::pair<int, gtsam::Pose3>> camPoses;
    tagGraph_.getCameraPoses(&camPoses, frameNum_);
    std::vector<PoseInfo> camPoseInfo;
    for (const auto &cp: camPoses) {
      const std::string frame_id = "cam_" + std::to_string(cp.first);
      camPoseInfo.push_back(PoseInfo(cp.second, t, frame_id));
    }
    broadcastTransforms(camPoseInfo);
  }

  void TagSlam::broadcastTagPoses(const ros::Time &t) {
    std::vector<std::pair<int, gtsam::Pose3>> tagPoses;
    tagGraph_.getTagWorldPoses(&tagPoses);
    std::vector<PoseInfo> tagPoseInfo;
    for (const auto &cp: tagPoses) {
      const std::string frame_id = "tag_" + std::to_string(cp.first);
      tagPoseInfo.push_back(PoseInfo(cp.second, t, frame_id));
    }
    broadcastTransforms(tagPoseInfo);
  }

  void TagSlam::broadcastTransforms(const std::vector<PoseInfo> &poses) {
    for (const auto &p: poses) {
      const auto &tf = gtsam_pose_to_tf(p.pose);
      tfBroadcaster_.sendTransform(tf::StampedTransform(tf, p.time, "world", p.frame_id));
    }
  }

  
  static void write_vec(std::ofstream &of,
                        const std::string &prefix,
                        double x, double y, double z) {
    of << prefix << "x: " << x << std::endl;
    of << prefix << "y: " << y << std::endl;
    of << prefix << "z: " << z << std::endl;
  }

  static void write_pose(std::ofstream &of, const std::string &prefix,
                         const gtsam::Pose3 &pose,
                         const Tag::PoseNoise &n) {
    gtsam::Vector r = gtsam::Rot3::Logmap(pose.rotation());
    gtsam::Vector t(pose.translation());
    const std::string pps = prefix + "  ";
    of << prefix << "center:" << std::endl;
    write_vec(of, pps, t(0), t(1), t(2));
    of << prefix << "rotvec:" << std::endl;
    write_vec(of, pps, r(0), r(1), r(2));
 
    gtsam::Vector nvec = n->sigmas();
    of << prefix << "position_noise:" << std::endl;
    write_vec(of, pps, nvec(3),nvec(4),nvec(5));
    of << prefix << "rotation_noise:" << std::endl;
    write_vec(of, pps, nvec(0),nvec(1),nvec(2));
  }

  void TagSlam::writeTagPoses(const std::string &poseFile) const {
    std::ofstream pf(poseFile);
    std::vector<std::pair<int, gtsam::Pose3>> tagPoses;
    tagGraph_.getTagWorldPoses(&tagPoses);
    std::map<int, gtsam::Pose3> idToPose;
    for (const auto &cp: tagPoses) {
      idToPose[cp.first] = cp.second;
    }
    if (!staticObjects_.empty()) {
      pf << "static_objects:" << std::endl;
    }
    for (const StaticObject &sobj : staticObjects_) {
      pf << " - " << sobj.name << ":" << std::endl;
      std::string pfix = "     ";
      pf << pfix << "pose:" << std::endl;;
      write_pose(pf, pfix + "  ", sobj.pose, sobj.noise);
      pf << pfix << "tags: " << std::endl;
      for (const auto &id: sobj.tagIds) {
        const Tag &tag = idToTag_.find(id)->second;
        pf << pfix << "- id: "   << tag.id << std::endl;
        pf << pfix << "  size: " << tag.size << std::endl;
        write_pose(pf, pfix + "  ", tag.pose, tag.noise);
      }
    }
  }

//#define DEBUG_TAG_POSE_INIT
  bool TagSlam::estimateInitialTagPose(int cam_idx, const gtsam::Pose3 &T_w_c,
                                       const gtsam::Point2 *corners,
                                       gtsam::Pose3 *tagPose) const {
    std::vector<gtsam::Point3> wp = Tag::get_object_corners(defaultTagSize_);
    std::vector<gtsam::Point2> ip;
    for (const auto i: irange(0, 4)) {
      ip.push_back(corners[i]);
    }

#ifdef DEBUG_TAG_POSE_INIT
    std::cout << "world points: " << std::endl;
    for (const auto &p: wp) {
      std::cout << p << std::endl;
    }
    std::cout << "image points: " << std::endl;
    for (const auto &p: ip) {
      std::cout << p << std::endl;
    }
#endif
    // find pose w.r.t camera
    const CameraIntrinsics &ci = cameras_[cam_idx].intrinsics;
    gtsam::Pose3 T_c_o = utils::get_init_pose(wp, ip, ci.intrinsics,
                                              ci.distortion_model,
                                              ci.distortion_coeffs);
#ifdef DEBUG_TAG_POSE_INIT    
    std::cout << "estimated T_c_o: " << std::endl << T_c_o << std::endl;
    std::cout << "camera T_w_c: "    << std::endl << T_w_c << std::endl;
#endif    
    // unknown tags belong to static object 0 by default
    const gtsam::Pose3 &T_s_w = staticObjects_[0].pose.inverse();
#ifdef DEBUG_TAG_POSE_INIT    
    std::cout << "object pose T_s_w: " << std::endl << T_s_w << std::endl;
#endif    
    const gtsam::Pose3 T_s_o = T_s_w * T_w_c * T_c_o;
    *tagPose = T_s_o;
#ifdef DEBUG_TAG_POSE_INIT
    std::cout << "tag pose T_s_o: " << std::endl << *tagPose << std::endl;
#endif
    return (true);
  }
  
  void TagSlam::callback1(TagArrayConstPtr const &tag0) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0};
    process(msg_vec);
  }

  void TagSlam::callback2(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1};
    process(msg_vec);
  }
  void TagSlam::callback3(TagArrayConstPtr const &tag0,
                          TagArrayConstPtr const &tag1,
                          TagArrayConstPtr const &tag2) {
    std::vector<TagArrayConstPtr> msg_vec = {tag0, tag1, tag2};
    process(msg_vec);
  }


  void TagSlam::playFromBag(const std::string &fname) {
    if (cameras_.size() != 1) {
      ROS_ERROR("bag playback is supported only for single camera!");
      ros::shutdown();
    }
    rosbag::Bag bag;
    bag.open(fname, rosbag::bagmode::Read);
    std::vector<std::string> topics = {cameras_[0].tagtopic};
    ROS_INFO_STREAM("playing from file: " << fname << " topic: " << topics[0]);
    double start_time(0);
    nh_.param<double>("start_time", start_time, 0);
    ros::Time t_start(start_time);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (const rosbag::MessageInstance &m: view) {
      boost::shared_ptr<TagArray> tags = m.instantiate<TagArray>();
      if (tags) {
        if (tags->header.stamp > t_start) {
          callback1(tags);
        }
      }
    }
    bag.close();
  }

}  // namespace
