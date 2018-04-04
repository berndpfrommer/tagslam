/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_TAGSLAM_H
#define TAGSLAM_TAGSLAM_H

#include "tagslam/camera.h"
#include "tagslam/tag_graph.h"
#include "tagslam/rigid_body.h"
#include "tagslam/distance_measurement.h"
#include "tagslam/initial_pose_graph.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
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
  typedef message_filters::sync_policies::ApproximateTime<TagArray, TagArray> SyncPolicy2;
  typedef message_filters::sync_policies::ApproximateTime<TagArray, TagArray, TagArray> SyncPolicy3;
  typedef message_filters::Synchronizer<SyncPolicy2> TimeSync2;
  typedef message_filters::Synchronizer<SyncPolicy3> TimeSync3;
  class TagSlam {
  public:
    TagSlam(const ros::NodeHandle &pnh);
    ~TagSlam();

    TagSlam(const TagSlam&) = delete;
    TagSlam& operator=(const TagSlam&) = delete;

    bool initialize();

    void callback1(TagArrayConstPtr const &tag0);
    void callback2(TagArrayConstPtr const &tag0, TagArrayConstPtr const &tag1);
    void callback3(TagArrayConstPtr const &tag0, TagArrayConstPtr const &tag1, TagArrayConstPtr const &tag2);
  private:
    struct PoseInfo {
      PoseInfo(const gtsam::Pose3 &p = gtsam::Pose3(), const ros::Time &t = ros::Time(0),
               const std::string &frid = "") : pose(p), time(t), frame_id(frid) {}
      gtsam::Pose3 pose;
      ros::Time    time;
      std::string  frame_id;
    };
    void process(const std::vector<TagArrayConstPtr> &msgvec);
    bool subscribe();
    void broadcastTransforms(const std::string &parentframe,
                             const std::vector<PoseInfo> &poses);
    void broadcastBodyPoses(const ros::Time &t);
    void broadcastCameraPoses(const ros::Time &t);
    void broadcastTagPoses(const ros::Time &t);
    bool estimateInitialTagPose(int cam_idx, const gtsam::Pose3 &T_w_c,
                                const gtsam::Point2 *corners,
                                gtsam::Pose3 *pose) const;
    RigidBodyPtr findBodyForTag(int tagId) const;

    void discoverTags(const std::vector<TagArrayConstPtr> &msgvec);
    unsigned int attachObservedTagsToBodies(const std::vector<TagArrayConstPtr> &msgvec);
    void detachObservedTagsFromBodies();

    bool estimateInitialTagPose(int cam_idx, const Tag &tag, gtsam::Pose3 *pose) const;

    PoseEstimate estimatePosePNP(int cam_idx,
                                 const std::vector<gtsam::Point3>&wpts,
                                 const std::vector<gtsam::Point2>&ipts) const;
    PoseEstimate estimatePoseHomography(int cam_idx,
                                        const std::vector<gtsam::Point3>&wpts,
                                        const std::vector<gtsam::Point2>&ipts) const;

    PoseEstimate poseFromPoints(int cam_idx,
                                const std::vector<gtsam::Point3> &wp,
                                const std::vector<gtsam::Point2> &ip,
                                bool pointsArePlanar = false) const;
    bool estimateTagPose(int cam_idx,
                         const gtsam::Pose3 &bodyPose,
                         const TagPtr &tag) const;
    void runOptimizer();
    PoseEstimate findCameraPose(int cam_idx, RigidBodyVec &rigidBodies,
                                bool bodiesMustHavePose) const;
    void findInitialCameraPoses();
    void findInitialBodyPoses();
    void findInitialDiscoveredTagPoses();
    void updatePosesFromGraph(unsigned int frame_num);
    void writeTagPoses(const std::string &poseFile) const;
    void playFromBag(const std::string &fname);
    bool readRigidBodies();
    void readDistanceMeasurements();
    bool isBadViewingAngle(const gtsam::Pose3 &p) const;
    // ----------------------------------------------------------
    typedef message_filters::Subscriber<TagArray> TagSubscriber;
    typedef std::unordered_map<int, TagPtr>       IdToTagMap;
    ros::Subscriber                               singleCamSub_;
    ros::Publisher                                camOdomPub_;
    std::vector<std::shared_ptr<TagSubscriber>>   sub_;
    std::unique_ptr<TimeSync2>                    approxSync2_;
    std::unique_ptr<TimeSync3>                    approxSync3_;
    ros::NodeHandle                               nh_;
    CameraVec                                     cameras_;
    TagGraph                                      tagGraph_;
    InitialPoseGraph                              initialPoseGraph_;
    IdToTagMap                                    allTags_;
    RigidBodyVec                                  staticBodies_;
    RigidBodyVec                                  dynamicBodies_;
    RigidBodyVec                                  allBodies_;
    RigidBodyPtr                                  defaultBody_;
    DistanceMeasurementVec                        distanceMeasurements_;
    unsigned int                                  frameNum_{0};
    tf::TransformBroadcaster                      tfBroadcaster_;
    std::string                                   tagPosesOutFile_;
    std::string                                   fixedFrame_;
    double                                        viewingAngleThreshold_;
  };

}

#endif
