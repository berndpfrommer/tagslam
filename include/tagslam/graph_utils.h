/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph.h"
#include "tagslam/body.h"
#include "tagslam/tag.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/camera.h"

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
    void add_tag(Graph *g,  const Tag &tag);
    void add_body(Graph *g, const Body &body);

    // Copy all vertices specified in srcfacs from the src graph
    // to the destination graph. Also copy all values that are used by
    // those factors, and pin them down with an absolute pose prior to
    // be their current values.
    void copy_subgraph(Graph *dest, const Graph &src,
                       const std::deque<VertexDesc> &srcfacs,
                       double absPriorPositionNoise,
                       double absPriorRotationNoise);
    // Look through the source graph for any values that are not yet
    // optimized in the destination graph. Initialize those values
    // in the destination graph, and add them to the optimizer.
    // Also add any destination graph factors to the optimizer if
    // they can now be optimized
    void initialize_from(Graph *destg, const Graph &srcg);

    // Removes from graph all factors and values listed. Optimized values
    // are kept! Caller must make sure that the graph is not
    // ill determined afterwards!
    void filter_graph(Graph *g,
                      const std::set<VertexDesc> &factorsToRemove,
                      const std::set<VertexDesc> &valuesToRemove);

    // convenience functions for retrieval of optimized poses
    bool get_optimized_pose(const Graph &g, const ros::Time &t,
                            const string &name, Transform *tf);
    bool get_optimized_pose(const Graph &g, const Camera &cam, Transform *tf);
    bool get_optimized_pose(const Graph &g, const ros::Time &t,
                            const Body &body, Transform *tf);
    bool get_optimized_pose(const Graph &g, const Tag &tag, Transform *tf);

    PoseWithNoise get_optimized_pose_with_noise(
      const Graph &g, const string &name);
    // graph plotting 
    void plot(const string &fname, const Graph *g);
    void plot_debug(const ros::Time &t, const string &tag, const Graph &g);
    }
}
