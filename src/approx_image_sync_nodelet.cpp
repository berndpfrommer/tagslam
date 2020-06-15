/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/approx_image_sync.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <memory>

namespace tagslam {
  class ApproxImageSyncNodelet: public nodelet::Nodelet {
  public:
    void onInit() override {
      node_.reset(new ApproxImageSync(getPrivateNodeHandle()));
      node_->initialize();
      ros::shutdown();
    }
  private:
    // ------ variables --------
    std::shared_ptr<ApproxImageSync> node_;
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tagslam::ApproxImageSyncNodelet, nodelet::Nodelet)
