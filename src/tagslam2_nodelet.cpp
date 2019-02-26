/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_slam2.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <memory>

namespace tagslam {
  class TagSlam2Nodelet: public nodelet::Nodelet {
  public:
    void onInit() override {
      node_.reset(new TagSlam2(getPrivateNodeHandle()));
      node_->initialize();
      ros::shutdown();
    }
  private:
    // ------ variables --------
    std::shared_ptr<TagSlam2> node_;
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tagslam::TagSlam2Nodelet, nodelet::Nodelet)
