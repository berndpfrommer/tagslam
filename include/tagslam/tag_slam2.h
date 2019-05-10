/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/tag_factory.h"
#include "tagslam/graph_manager.h"
#include "tagslam/graph_updater.h"
#include "tagslam/graph.h"
#include "tagslam/camera2.h"
#include "tagslam/profiler.h"
#include "tagslam/odometry_processor.h"
#include "tagslam/measurements/measurements.h"

#include <flex_sync/sync.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>


#include <string>
#include <unordered_map>
#include <map>
#include <set>


namespace tagslam {
  class TagSlam2: public TagFactory {
    using Apriltag = apriltag_msgs::Apriltag;
    using TagArray = apriltag_msgs::ApriltagArrayStamped;
    using TagArrayPtr = TagArray::Ptr;
    using TagArrayConstPtr = TagArray::ConstPtr;
    using Odometry = nav_msgs::Odometry;
    using OdometryConstPtr = nav_msgs::OdometryConstPtr;
    using Image = sensor_msgs::Image;
    using ImageConstPtr = sensor_msgs::ImageConstPtr;
    using CompressedImage = sensor_msgs::CompressedImage;
    using CompressedImageConstPtr = sensor_msgs::CompressedImageConstPtr;
    using string = std::string;
  public:
    TagSlam2(const ros::NodeHandle &nh);
    TagSlam2(const TagSlam2&) = delete;
    TagSlam2& operator=(const TagSlam2&) = delete;

    // inherited from TagFactory
    Tag2ConstPtr findTag(int tagId) override;

    // ------ own methods
    bool initialize();

    template<typename S>
    void processBag(S *sync, rosbag::View *view) {
      for (const rosbag::MessageInstance &m: *view) {
         typename S::Type1::ConstPtr t1 = m.instantiate<typename S::Type1>();
        if (t1) {    sync->process(m.getTopic(), t1); } else {
          typename S::Type2::ConstPtr t2 = m.instantiate<typename S::Type2>();
          if (t2) {  sync->process(m.getTopic(), t2); } else {
            typename S::Type3::ConstPtr t3 =m.instantiate<typename S::Type3>();
            if (t3) { sync->process(m.getTopic(), t3); }
          }
        }
        if (frameNum_ > maxFrameNum_ || !ros::ok()) {
          break;
        }
      }
    }
    void syncCallback(const std::vector<TagArrayConstPtr> &msgvec1,
                      const std::vector<ImageConstPtr> &msgvec2,
                      const std::vector<OdometryConstPtr> &msgvec3);
    void syncCallbackCompressed(
      const std::vector<TagArrayConstPtr> &msgvec1,
      const std::vector<CompressedImageConstPtr> &msgvec2,
      const std::vector<OdometryConstPtr> &msgvec3);
  private:
    struct ReMap {
      ReMap(int i, ros::Time ts, ros::Time te) :
        remappedId(i), startTime(ts), endTime(te) {
      };
      int       remappedId;
      ros::Time startTime;
      ros::Time endTime;
    };
    typedef std::unordered_map<int, Tag2ConstPtr> TagMap;

    void readBodies(XmlRpc::XmlRpcValue config);
    void readDistanceMeasurements();
    void readRemap(XmlRpc::XmlRpcValue config);
    void readSquash(XmlRpc::XmlRpcValue config);
    void playFromBag(const std::string &fname);
    void processOdom(const std::vector<OdometryConstPtr> &odomMsg,
                     std::vector<VertexDesc> *factors);
    std::vector<std::vector<string>> makeTopics(rosbag::Bag *bag) const;
    void setupOdom(const std::vector<OdometryConstPtr> &odomMsgs);
    void processTagsAndOdom(const std::vector<TagArrayConstPtr> &tagmsgs,
                            const std::vector<OdometryConstPtr> &odommsgs);
    void publishTagAndBodyTransforms(const ros::Time &t, tf::tfMessage *tfMsg);
    void publishOriginalTagTransforms(const ros::Time &t, tf::tfMessage *tfMsg);
    void publishCameraTransforms(const ros::Time &t, tf::tfMessage *tfMsg);

    void publishTransforms(const ros::Time &t, bool orig = false);
    void publishBodyOdom(const ros::Time &t);
    void sleep(double dt) const;
    void processTags(const std::vector<TagArrayConstPtr> &tagMsgs,
                     std::vector<VertexDesc> *factors);
    std::vector<Tag2ConstPtr> findTags(const std::vector<Apriltag> &ta);
    bool anyTagsVisible(const std::vector<TagArrayConstPtr> &tagmsgs);
    void publishAll(const ros::Time &t);
    bool replay(std_srvs::Trigger::Request& req,
                std_srvs::Trigger::Response &res);
    void writeCameraPoses(const string &fname) const;
    void writePoses(const string &fname) const;
    void writeTagDiagnostics(const string &fname) const;
    void writeTimeDiagnostics(const string &fname) const;
    void writeDistanceDiagnostics(const string &fname) const;
    void writeErrorMap(const string &fname) const;
    void writeTagCorners(const ros::Time &t, int camIdx, const Tag2ConstPtr &tag,
                         const geometry_msgs::Point *img_corners);

    void remapAndSquash(std::vector<TagArrayConstPtr> *remapped,
                        const std::vector<TagArrayConstPtr> &orig);
    void applyDistanceMeasurements();

    // ------ variables --------
    ros::NodeHandle      nh_;
    GraphPtr             graph_;
    GraphManager         graphManager_;
    GraphUpdater         graphUpdater_;
    Camera2Vec           cameras_;
    BodyVec              bodies_;
    BodyVec              nonstaticBodies_;
    BodyPtr              defaultBody_;
    ros::Publisher       clockPub_;
    std::vector<ros::Publisher> odomPub_;
    string               fixedFrame_;
    bool                 writeDebugImages_;
    bool                 hasCompressedImages_;
    int                  frameNum_{0};
    int                  maxFrameNum_{1000000};
    double               playbackRate_{1.0};
    std::vector<cv::Mat> images_;
    std::vector<OdometryProcessor> odomProcessors_;
    tf::TransformBroadcaster tfBroadcaster_;
    ros::ServiceServer   service_;
    TagMap               tagMap_;
    Profiler             profiler_;
    std::list<ros::Time> times_;
    rosbag::Bag          outBag_;
    std::ofstream        tagCornerFile_;
    std::string          outBagName_;
    std::unordered_map<int, std::vector<ReMap>>  tagRemap_;
    std::map<ros::Time, std::set<int>> squash_;
    std::vector<MeasurementsPtr> measurements_;
  };
}
