/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/factor/factor.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/geometry.h"
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

namespace tagslam {
  class Tag2;
  class Camera2;
  namespace factor {
    class TagProjection: public Factor {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      TagProjection(const ros::Time     &t  = ros::Time(0),
                    const std::shared_ptr<const Camera2> &cam =
                    std::shared_ptr<Camera2>(),
                    const std::shared_ptr<const Tag2> &tag =
                    std::shared_ptr<Tag2>(),
                    const geometry_msgs::Point *imgCorn = NULL,
                    const std::string   &name = "");
      std::string getLabel() const override;
      const Eigen::Matrix<double, 4,2> &getImageCorners() const { return (imgCorners_); }
      const std::shared_ptr<const Camera2> getCamera() const { return (cam_); }
      const std::shared_ptr<const Tag2> getTag() const { return (tag_); }
    private:
      ros::Time                            time_;
      const std::shared_ptr<const Camera2> cam_;
      const std::shared_ptr<const Tag2>    tag_;
      Eigen::Matrix<double, 4, 2>          imgCorners_;
    };
  }
  typedef std::shared_ptr<factor::TagProjection> TagProjectionFactorPtr;
  typedef std::shared_ptr<const factor::TagProjection> TagProjectionFactorConstPtr;
}
