/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/factor/tag_projection.h"
#include <geometry_msgs/Point.h>
#include <string>
#include <sstream>

namespace tagslam {
  namespace factor {
    TagProjection::TagProjection(const ros::Time &t,
                                 const std::shared_ptr<const Camera2> &cam,
                                 const std::shared_ptr<const Tag2> &tag,
                                 const geometry_msgs::Point *imgCorn,
                                 const std::string   &name) :
      Factor(name), time_(t), cam_(cam), tag_(tag) {
      imgCorners_[0] = Point2d(imgCorn[0].x, imgCorn[0].y);
      imgCorners_[1] = Point2d(imgCorn[1].x, imgCorn[1].y);
      imgCorners_[2] = Point2d(imgCorn[2].x, imgCorn[2].y);
      imgCorners_[3] = Point2d(imgCorn[3].x, imgCorn[3].y);
    }
    std::string TagProjection::getLabel() const {
      std::stringstream ss;
      ss << "proj:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }
  } // namespace factor
}  // namespace tagslam
