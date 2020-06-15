/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/approx_image_sync.h"

#include <boost/range/irange.hpp>
#include <std_msgs/Time.h>
#include <stdexcept>

namespace tagslam {
  using boost::irange;

  ApproxImageSync::ApproxImageSync(const ros::NodeHandle &nh) : nh_(nh) {
  }

  void ApproxImageSync::initialize() {
    std::vector<std::string> inTopics, outTopics;
    if (!nh_.getParam("in_topics", inTopics)) {
      ROS_ERROR("no in_topics parameter found!");
      throw (std::invalid_argument("in_topics missing"));
    }
    if (inTopics.empty()) {
      ROS_ERROR("no in_topics specified!");
      throw (std::invalid_argument("in_topics empty"));
    }
    if (!nh_.getParam("out_topics", outTopics)) {
      ROS_ERROR("no out_topics parameter found!");
      throw (std::invalid_argument("out_topics missing"));
    }
    if (inTopics.size() != outTopics.size()) {
      ROS_ERROR("num out topics must match num in topics!");
      throw (std::invalid_argument("num topics mismatch"));
    }
    int queue_size;
    nh_.param<int>("queue_size", queue_size, 1);
    for (const auto &topic: inTopics) {
      sub_.push_back(std::make_shared<message_filters::Subscriber<Image>>(
                        nh_, topic, queue_size));
    }
    const int sync_qs = queue_size; // need not necessarily be tied
    switch (inTopics.size()) {
    case 1:
      ROS_ERROR("must have > 1 image topic");
      throw std::invalid_argument("must have > 1 image topic");
      break;
    case 2:
      sync2_.reset(new Sync2(ApproxPolicy2(sync_qs), *sub_[0], *sub_[1]));
      sync2_->registerCallback(boost::bind(
                                &ApproxImageSync::callback2, this, _1, _2));
      break;
    case 3:
      sync3_.reset(new Sync3(ApproxPolicy3(sync_qs), *sub_[0], *sub_[1], *sub_[2]));
      sync3_->registerCallback(boost::bind(
                                &ApproxImageSync::callback3, this, _1, _2, _3));
      break;
    case 4:
      sync4_.reset(new Sync4(ApproxPolicy4(sync_qs),
                             *sub_[0], *sub_[1], *sub_[2], *sub_[3]));
      sync4_->registerCallback(boost::bind(
                                 &ApproxImageSync::callback4, this,
                                 _1, _2, _3, _4));
      break;
    case 5:
      sync5_.reset(new Sync5(ApproxPolicy5(sync_qs),
                             *sub_[0], *sub_[1], *sub_[2], *sub_[3], *sub_[4]));
      sync5_->registerCallback(boost::bind(
                                 &ApproxImageSync::callback5, this,
                                 _1, _2, _3, _4, _5));
      break;
    case 6:
      sync6_.reset(new Sync6(ApproxPolicy6(sync_qs),
                             *sub_[0], *sub_[1], *sub_[2], *sub_[3], *sub_[4], *sub_[5]));
      sync6_->registerCallback(boost::bind(
                                 &ApproxImageSync::callback6, this,
                                 _1, _2, _3, _4, _5, _6));
      break;
    case 7:
      sync7_.reset(new Sync7(ApproxPolicy7(sync_qs),
                             *sub_[0], *sub_[1], *sub_[2], *sub_[3],
                             *sub_[4], *sub_[5], *sub_[6]));
      sync7_->registerCallback(boost::bind(
                                 &ApproxImageSync::callback7, this,
                                 _1, _2, _3, _4, _5, _6, _7));
      break;
    case 8:
      sync8_.reset(new Sync8(ApproxPolicy8(sync_qs),
                             *sub_[0], *sub_[1], *sub_[2], *sub_[3],
                             *sub_[4], *sub_[5], *sub_[6], *sub_[7]));
      sync8_->registerCallback(boost::bind(
                                 &ApproxImageSync::callback8, this,
                                 _1, _2, _3, _4, _5, _6, _7, _8));
      break;
    default:
      ROS_ERROR("too many image topics!");
      throw std::invalid_argument("too many image topics");
      break;
    }
    for (const auto &topic: outTopics) {
      pub_.push_back(nh_.advertise<Image>(topic, queue_size));
    }
  }

  static sensor_msgs::ImageConstPtr modify_stamp(
    const ros::Time &stamp,
    const sensor_msgs::ImageConstPtr &imgMsg) {
    if (stamp == imgMsg->header.stamp) {
      return (imgMsg);
    }
    sensor_msgs::ImagePtr modImg(new sensor_msgs::Image(*imgMsg));
    modImg->header.stamp = stamp;
    return (modImg);
  }

  void ApproxImageSync::callback2(const ImageConstPtr &img0,
                                  const ImageConstPtr &img1) {
    const ros::Time stamp = img0->header.stamp;
    pub_[0].publish(img0);
    pub_[1].publish(modify_stamp(stamp, img1));
  }

  void ApproxImageSync::callback3(const ImageConstPtr &img0,
                                  const ImageConstPtr &img1,
                                  const ImageConstPtr &img2) {
    const ros::Time stamp = img0->header.stamp;
    pub_[0].publish(img0);
    pub_[1].publish(modify_stamp(stamp, img1));
    pub_[2].publish(modify_stamp(stamp, img2));
  }

  void ApproxImageSync::callback4(const ImageConstPtr &img0,
                                  const ImageConstPtr &img1,
                                  const ImageConstPtr &img2,
                                  const ImageConstPtr &img3) {
    const ros::Time stamp = img0->header.stamp;
    pub_[0].publish(img0);
    pub_[1].publish(modify_stamp(stamp, img1));
    pub_[2].publish(modify_stamp(stamp, img2));
    pub_[3].publish(modify_stamp(stamp, img3));
  }

  void ApproxImageSync::callback5(const ImageConstPtr &img0,
                                  const ImageConstPtr &img1,
                                  const ImageConstPtr &img2,
                                  const ImageConstPtr &img3,
                                  const ImageConstPtr &img4) {
    const ros::Time stamp = img0->header.stamp;
    pub_[0].publish(img0);
    pub_[1].publish(modify_stamp(stamp, img1));
    pub_[2].publish(modify_stamp(stamp, img2));
    pub_[3].publish(modify_stamp(stamp, img3));
    pub_[4].publish(modify_stamp(stamp, img4));
  }

  void ApproxImageSync::callback6(const ImageConstPtr &img0,
                                  const ImageConstPtr &img1,
                                  const ImageConstPtr &img2,
                                  const ImageConstPtr &img3,
                                  const ImageConstPtr &img4,
                                  const ImageConstPtr &img5) {
    const ros::Time stamp = img0->header.stamp;
    pub_[0].publish(img0);
    pub_[1].publish(modify_stamp(stamp, img1));
    pub_[2].publish(modify_stamp(stamp, img2));
    pub_[3].publish(modify_stamp(stamp, img3));
    pub_[4].publish(modify_stamp(stamp, img4));
    pub_[5].publish(modify_stamp(stamp, img5));
  }

  void ApproxImageSync::callback7(const ImageConstPtr &img0,
                                  const ImageConstPtr &img1,
                                  const ImageConstPtr &img2,
                                  const ImageConstPtr &img3,
                                  const ImageConstPtr &img4,
                                  const ImageConstPtr &img5,
                                  const ImageConstPtr &img6) {
    const ros::Time stamp = img0->header.stamp;
    pub_[0].publish(img0);
    pub_[1].publish(modify_stamp(stamp, img1));
    pub_[2].publish(modify_stamp(stamp, img2));
    pub_[3].publish(modify_stamp(stamp, img3));
    pub_[4].publish(modify_stamp(stamp, img4));
    pub_[5].publish(modify_stamp(stamp, img5));
    pub_[6].publish(modify_stamp(stamp, img6));
  }

  void ApproxImageSync::callback8(const ImageConstPtr &img0,
                                  const ImageConstPtr &img1,
                                  const ImageConstPtr &img2,
                                  const ImageConstPtr &img3,
                                  const ImageConstPtr &img4,
                                  const ImageConstPtr &img5,
                                  const ImageConstPtr &img6,
                                  const ImageConstPtr &img7) {
    const ros::Time stamp = img0->header.stamp;
    pub_[0].publish(img0);
    pub_[1].publish(modify_stamp(stamp, img1));
    pub_[2].publish(modify_stamp(stamp, img2));
    pub_[3].publish(modify_stamp(stamp, img3));
    pub_[4].publish(modify_stamp(stamp, img4));
    pub_[5].publish(modify_stamp(stamp, img5));
    pub_[6].publish(modify_stamp(stamp, img6));
    pub_[7].publish(modify_stamp(stamp, img7));
  }

  void ApproxImageSync::callback9(const ImageConstPtr &img0,
                                  const ImageConstPtr &img1,
                                  const ImageConstPtr &img2,
                                  const ImageConstPtr &img3,
                                  const ImageConstPtr &img4,
                                  const ImageConstPtr &img5,
                                  const ImageConstPtr &img6,
                                  const ImageConstPtr &img7,
                                  const ImageConstPtr &img8) {
    const ros::Time stamp = img0->header.stamp;
    pub_[0].publish(img0);
    pub_[1].publish(modify_stamp(stamp, img1));
    pub_[2].publish(modify_stamp(stamp, img2));
    pub_[3].publish(modify_stamp(stamp, img3));
    pub_[4].publish(modify_stamp(stamp, img4));
    pub_[5].publish(modify_stamp(stamp, img5));
    pub_[6].publish(modify_stamp(stamp, img6));
    pub_[7].publish(modify_stamp(stamp, img7));
    pub_[8].publish(modify_stamp(stamp, img8));
  }

}  // end of namespace

