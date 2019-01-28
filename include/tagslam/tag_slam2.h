/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/graph.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <string>

namespace tagslam {
  class TagSlam2: public nodelet::Nodelet {
  public:
    TagSlam2();
    ~TagSlam2();

    TagSlam2(const TagSlam2&) = delete;
    TagSlam2& operator=(const TagSlam2&) = delete;

    void onInit() override;

  private:
    void makeGraph();
    bool initialize();
    void readBodies();
    // ------ variables --------
    ros::NodeHandle nh_;
    Graph           graph_;
    std::string     paramPrefix_;
  };
}
