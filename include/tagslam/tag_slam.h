/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_TAGSLAM_H
#define TAGSLAM_TAGSLAM_H

#include "tagslam/camera.h"
#include "tagslam/tag_graph.h"
#include "tagslam/rigid_body.h"
#include "tagslam/distance_measurement.h"
#include "tagslam/position_measurement.h"
#include "tagslam/initial_pose_graph.h"
#include "tagslam/profiler.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <vector>
#include <memory>
#include <map>
#include <set>
#include <string>
#include <unordered_map>

namespace tagslam {
  using TagArray = apriltag_msgs::ApriltagArrayStamped;
  using TagArrayConstPtr = TagArray::ConstPtr;
  using Image = sensor_msgs::Image;
  using ImageConstPtr = sensor_msgs::ImageConstPtr;
  using CompressedImage = sensor_msgs::CompressedImage;
  using CompressedImageConstPtr = sensor_msgs::CompressedImageConstPtr;
  typedef message_filters::sync_policies::ApproximateTime<TagArray, TagArray> SyncPolicy2;
  typedef message_filters::sync_policies::ApproximateTime<TagArray, TagArray, TagArray> SyncPolicy3;
  typedef message_filters::sync_policies::ApproximateTime<TagArray, TagArray, TagArray, TagArray> SyncPolicy4;
  typedef message_filters::sync_policies::ApproximateTime<TagArray, TagArray, TagArray, TagArray, TagArray> SyncPolicy5;
  typedef message_filters::sync_policies::ApproximateTime<TagArray, TagArray, TagArray, TagArray, TagArray, TagArray> SyncPolicy6;
  typedef message_filters::sync_policies::ApproximateTime<TagArray, TagArray, TagArray, TagArray, TagArray, TagArray, TagArray> SyncPolicy7;
  typedef message_filters::sync_policies::ApproximateTime<TagArray, TagArray, TagArray, TagArray, TagArray, TagArray, TagArray, TagArray> SyncPolicy8;
  typedef message_filters::Synchronizer<SyncPolicy2> TimeSync2;
  typedef message_filters::Synchronizer<SyncPolicy3> TimeSync3;
  typedef message_filters::Synchronizer<SyncPolicy4> TimeSync4;
  typedef message_filters::Synchronizer<SyncPolicy5> TimeSync5;
  typedef message_filters::Synchronizer<SyncPolicy6> TimeSync6;
  typedef message_filters::Synchronizer<SyncPolicy7> TimeSync7;
  typedef message_filters::Synchronizer<SyncPolicy8> TimeSync8;
  class TagSlam {
  public:
    TagSlam(const ros::NodeHandle &pnh);
    ~TagSlam();

    TagSlam(const TagSlam&) = delete;
    TagSlam& operator=(const TagSlam&) = delete;

    bool initialize();
    void callback1(TagArrayConstPtr const &tag0);
    void callback2(TagArrayConstPtr const &tag0,
                   TagArrayConstPtr const &tag1);
    void callback3(TagArrayConstPtr const &tag0,
                   TagArrayConstPtr const &tag1,
                   TagArrayConstPtr const &tag2);
    void callback4(TagArrayConstPtr const &tag0,
                   TagArrayConstPtr const &tag1,
                   TagArrayConstPtr const &tag2,
                   TagArrayConstPtr const &tag3);
    void callback5(TagArrayConstPtr const &tag0,
                   TagArrayConstPtr const &tag1,
                   TagArrayConstPtr const &tag2,
                   TagArrayConstPtr const &tag3,
                   TagArrayConstPtr const &tag4);
    void callback6(TagArrayConstPtr const &tag0,
                   TagArrayConstPtr const &tag1,
                   TagArrayConstPtr const &tag2,
                   TagArrayConstPtr const &tag3,
                   TagArrayConstPtr const &tag4,
                   TagArrayConstPtr const &tag5);
    void callback7(TagArrayConstPtr const &tag0,
                   TagArrayConstPtr const &tag1,
                   TagArrayConstPtr const &tag2,
                   TagArrayConstPtr const &tag3,
                   TagArrayConstPtr const &tag4,
                   TagArrayConstPtr const &tag5,
                   TagArrayConstPtr const &tag6);
    void callback8(TagArrayConstPtr const &tag0,
                   TagArrayConstPtr const &tag1,
                   TagArrayConstPtr const &tag2,
                   TagArrayConstPtr const &tag3,
                   TagArrayConstPtr const &tag4,
                   TagArrayConstPtr const &tag5,
                   TagArrayConstPtr const &tag6,
                   TagArrayConstPtr const &tag7);
  private:
    typedef std::vector<std::pair<gtsam::Point3, bool> > PointVector;
    struct PoseInfo {
      PoseInfo(const gtsam::Pose3 &p = gtsam::Pose3(), const ros::Time &t = ros::Time(0),
               const std::string &pfrid = "",
               const std::string &frid  = "") :
        pose(p), time(t), parent_frame_id(pfrid), frame_id(frid) {}
      gtsam::Pose3 pose;
      ros::Time    time;
      std::string  parent_frame_id;
      std::string  frame_id;
    };
    void processTags(const std::vector<TagArrayConstPtr> &msgvec);
    void processTagsAndImages(const std::vector<TagArrayConstPtr> &msgvec1,
                              const std::vector<ImageConstPtr> &msgvec2);
    void processTagsAndCompressedImages(const std::vector<TagArrayConstPtr> &msgvec1,
                                        const std::vector<CompressedImageConstPtr> &msgvec2);

    bool subscribe();
    void broadcastTransforms(const std::vector<PoseInfo> &poses);
    void broadcastBodyPoses(const ros::Time &t);
    void broadcastCameraPoses(const ros::Time &t);
    void broadcastTagPoses(const ros::Time &t);
    bool estimateInitialTagPose(int cam_idx, const gtsam::Pose3 &T_w_c,
                                const gtsam::Point2 *corners,
                                gtsam::Pose3 *pose) const;
    RigidBodyPtr findBodyForTag(int tagId, int bits) const;

    void discoverTags(const std::vector<TagArrayConstPtr> &msgvec);
    unsigned int attachObservedTagsToBodies(const std::vector<TagArrayConstPtr> &msgvec);
    void detachObservedTagsFromBodies();

    bool estimateInitialTagPose(int cam_idx, const Tag &tag, gtsam::Pose3 *pose) const;

    PoseEstimate estimatePosePNP(int cam_idx,
                                 const std::vector<gtsam::Point3>&wpts,
                                 const std::vector<gtsam::Point2>&ipts) const;
    PoseEstimate poseFromPoints(int cam_idx,
                                const std::vector<gtsam::Point3> &wp,
                                const std::vector<gtsam::Point2> &ip,
                                bool pointsArePlanar = false) const;
    bool estimateTagPose(int cam_idx,
                         const gtsam::Pose3 &bodyPose,
                         const TagPtr &tag) const;
    PoseEstimate estimateBodyPose(const RigidBodyConstPtr &rb) const;
    void computeProjectionError();
    void runOptimizer();
    void finalize();
    PoseEstimate findCameraPose(int cam_idx, const RigidBodyConstVec &rigidBodies,
                                bool inWorldFrame) const;
    PoseEstimate guessCameraWorldPose(unsigned int cam_idx) const;
    void findInitialCameraAndRigPoses();
    void findInitialBodyPoses();
    void findInitialDiscoveredTagPoses();
    std::vector<int> findCamerasWithKnownWorldPose() const;
    void updatePosesFromGraph(unsigned int frame_num);
    void writeBodyPoses(const std::string &poseFile) const;
    void writeTagWorldPoses(const std::string &poseFile, unsigned int frameNum) const;
    void writeCameraPoses(const std::string &fname) const;
    void playFromBag(const std::string &fname);
    bool readRigidBodies();
    void readMeasurements(const std::string &type);
    bool attachCamerasToBodies();
    void invalidateDynamicPoses();
    void applyDistanceMeasurements();
    void applyPositionMeasurements();
    bool isBadViewingAngle(const gtsam::Pose3 &p) const;
    void writeMeasurements(const std::string &fname,
                           const PointVector &dist, const PointVector &pos) const;
    void writeDistanceMeasurements(std::ostream &f, const PointVector &dist) const;
    void writePositionMeasurements(std::ostream &f, const PointVector &pos) const;
    void printDistanceErrors(const PointVector &dist) const;
    void printPositionErrors(const PointVector &pos) const;
    PointVector getPositions() const;
    PointVector getDistances() const;
    // ----------------------------------------------------------
    typedef message_filters::Subscriber<TagArray> TagSubscriber;
    typedef std::unordered_map<int, TagPtr>       IdToTagMap;
    ros::Subscriber                               singleCamSub_;
    std::vector<ros::Publisher>                   camOdomPub_;
    std::vector<ros::Publisher>                   bodyOdomPub_;
    std::vector<std::shared_ptr<TagSubscriber>>   sub_;
    std::unique_ptr<TimeSync2>                    approxSync2_;
    std::unique_ptr<TimeSync3>                    approxSync3_;
    std::unique_ptr<TimeSync4>                    approxSync4_;
    std::unique_ptr<TimeSync5>                    approxSync5_;
    std::unique_ptr<TimeSync6>                    approxSync6_;
    std::unique_ptr<TimeSync7>                    approxSync7_;
    std::unique_ptr<TimeSync8>                    approxSync8_;
    
    ros::NodeHandle                               nh_;
    CameraVec                                     cameras_;
    std::vector<cv::Mat>                          images_;
    TagGraph                                      tagGraph_;
    InitialPoseGraph                              initialPoseGraph_;
    IdToTagMap                                    allTags_;
    RigidBodyVec                                  staticBodies_;
    RigidBodyVec                                  dynamicBodies_;
    RigidBodyVec                                  allBodies_;
    RigidBodyPtr                                  defaultBody_;
    DistanceMeasurementVec                        distanceMeasurements_;
    DistanceMeasurementVec                        unappliedDistanceMeasurements_;
    PositionMeasurementVec                        positionMeasurements_;
    PositionMeasurementVec                        unappliedPositionMeasurements_;
    unsigned int                                  frameNum_{0};
    int                                           maxFrameNum_{1000000};
    bool                                          writeDebugImages_{false};
    bool                                          hasCompressedImages_{false};
    tf::TransformBroadcaster                      tfBroadcaster_;
    std::string                                   paramPrefix_;
    std::string                                   bodyPosesOutFile_;
    std::string                                   tagWorldPosesOutFile_;
    std::string                                   cameraPosesOutFile_;
    std::string                                   measurementsOutFile_;
    std::string                                   fixedFrame_;
    double                                        viewingAngleThreshold_;
    double                                        initBodyPoseMaxError_;
    double                                        maxInitErr_{0.02};
    Profiler                                      profiler_;
  };

}

#endif
