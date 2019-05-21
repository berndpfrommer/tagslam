/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph.h"
#include "tagslam/body.h"
#include "tagslam/tag2.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/camera2.h"

#include <ros/ros.h>

#include <string>

#pragma once

namespace tagslam {
  namespace graph_utils {
    using std::string;
    // convenience functions for adding to graph
    void add_pose_maybe_with_prior(Graph *g, const ros::Time &t,
                                   const string &name,
                                   const PoseWithNoise &pwn, bool isCamPose);
    void add_tag(Graph *g,  const Tag2 &tag);
    void add_body(Graph *g, const Body &body);

    // convenience functions for retrieval of optimized poses
    bool get_optimized_pose(const Graph &g, const ros::Time &t,
                            const string &name, Transform *tf);
    bool get_optimized_pose(const Graph &g, const Camera2 &cam, Transform *tf);
    bool get_optimized_pose(const Graph &g, const ros::Time &t,
                            const Body &body, Transform *tf);
    bool get_optimized_pose(const Graph &g, const Tag2 &tag, Transform *tf);

    PoseWithNoise get_optimized_camera_pose(const Graph &g,
                                            const Camera2 &cam);

  }
}
