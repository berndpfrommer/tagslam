/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_slam.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <memory>

namespace tagslam {
  class TagSlamNodelet: public nodelet::Nodelet {
  public:
    void onInit() override {
      node_.reset(new TagSlam(getPrivateNodeHandle()));
      node_->initialize();
      ros::shutdown();
    }
  private:
    // ------ variables --------
    std::shared_ptr<TagSlam> node_;
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tagslam::TagSlamNodelet, nodelet::Nodelet)
