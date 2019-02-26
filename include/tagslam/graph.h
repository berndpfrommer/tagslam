/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/boost_graph.h"
#include "tagslam/body.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/value_key.h"

#include <ros/ros.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>
#include <memory>

#pragma once

namespace tagslam {
  class Optimizer;
  namespace value {
    class Pose;
  }
  class Graph {
  public:
    Graph();
    ~Graph();
    void setOptimizer(Optimizer *opt) { optimizer_ = opt; }
    std::shared_ptr<value::Pose>
    addPoseWithPrior(const ros::Time &t, const std::string &name,
                     const PoseWithNoise &pn,
                     BoostGraphVertex *vd);
    bool getBodyPose(const ros::Time &t,
                     const BodyConstPtr &body, Transform *tf) const;
    bool getTagPose(const ros::Time &t,
                    const Tag2ConstPtr &tag, Transform *tf) const;
    void addBody(const Body &body);
    void addBodyPoseDelta(const ros::Time &tPrev, const ros::Time &tCurr,
                          const BodyConstPtr &body,
                          const PoseWithNoise &deltaPose);
    void setOptimizeFullGraph(bool fg) { optimizeFullGraph_ = fg; }
    void plotDebug(const ros::Time &t, const std::string &tag);
    void optimize();
    void test();

  private:
    // lookup table entry
    struct Entry {
      Entry(const ros::Time &t = ros::Time(0),
            const BoostGraphVertex &v = BoostGraphVertex(),
            const std::shared_ptr<value::Pose> &p =
            std::shared_ptr<value::Pose>()) : time(t), vertex(v), pose(p) {}
      ros::Time         time{0};  // last body pose update entered
      BoostGraphVertex  vertex;   // last pose value vertex
      std::shared_ptr<value::Pose> pose;     // also has optimizer key
    };
    BoostGraphVertex addTag(const Tag2 &tag);

    // ------ variables --------------
    typedef std::unordered_map<int, BoostGraphVertex> IntToVertexMap;
    BoostGraph         graph_;
    bool               optimizeFullGraph_;
    Optimizer         *optimizer_;
    std::vector<Entry> bodyLookupTable_;
    IntToVertexMap     tagIdToPoseVertex_;
  };
}
