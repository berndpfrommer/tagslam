/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph_utils.h"


namespace tagslam {
  namespace graph_utils {
    using std::string;
    void add_pose_maybe_with_prior(Graph *g, const ros::Time &t,
                                   const string &name,
                                   const PoseWithNoise &pwn, bool isCamPose) {
      const VertexDesc v = g->addPose(t, name, isCamPose);
      if (pwn.isValid()) { // have valid pose prior
        // can already add pose to optimizer
        g->getPoseVertex(v)->addToOptimizer(pwn.getPose(), g);
        // now add factor to graph and optimizer
        const std::shared_ptr<factor::AbsolutePosePrior>
          fac(new factor::AbsolutePosePrior(t, pwn, name));
        fac->addToGraph(fac, g);
        fac->addToOptimizer(g);
      }
    }

    void add_tag(Graph *g, const Tag2 &tag) {
      const string name = Graph::tag_name(tag.getId());
      const ros::Time t0(0);
      add_pose_maybe_with_prior(g, t0, name, tag.getPoseWithNoise(), false);
    }

    void add_body(Graph *g, const Body &body) {
      // add body pose as vertex
      if (body.isStatic()) {
        const ros::Time t0(0);
        const string name = Graph::body_name(body.getName());
        add_pose_maybe_with_prior(g, t0, name, body.getPoseWithNoise(), false);
      } 
      // add associated tags as vertices
      for (const auto &tag: body.getTags()) {
        add_tag(g, *tag);
      }
      ROS_INFO_STREAM("added body " << body.getName() << " with "
                      << body.getTags().size() << " tags");
    }

    bool get_optimized_pose(const Graph &g, const ros::Time &t,
                            const string &name, Transform *tf) {
      const VertexDesc v = g.findPose(t, name);
      if (!Graph::is_valid(v) || !g.isOptimized(v)) {
        return (false);
      }
      *tf = g.getOptimizedPose(v);
      return (true);
    }

    bool get_optimized_pose(const Graph &g, const Camera2 &cam,
                            Transform *tf) {
      return (get_optimized_pose(g, ros::Time(0),
                                 Graph::cam_name(cam.getName()), tf));
    }

    bool get_optimized_pose(const Graph &g, const ros::Time &t,
                            const Body &body, Transform *tf) {
      return (get_optimized_pose(g, t, Graph::body_name(body.getName()), tf));
    }

    bool get_optimized_pose(const Graph &g, const Tag2 &tag, Transform *tf) {
      return (get_optimized_pose(g, ros::Time(0),
                                 Graph::tag_name(tag.getId()), tf));
    }
    
    PoseWithNoise get_optimized_camera_pose(const Graph &g,
                                            const Camera2 &cam) {
      PoseWithNoise pwn;
      const VertexDesc v = g.findPose(ros::Time(0),
                                      Graph::cam_name(cam.getName()));
      if (!Graph::is_valid(v) || !g.isOptimized(v)) {
        return (PoseWithNoise());
      }
      return (PoseWithNoise(g.getOptimizedPose(v), g.getPoseNoise(v), true));
    }

  }  // end of namespace
}  // end of namespace
