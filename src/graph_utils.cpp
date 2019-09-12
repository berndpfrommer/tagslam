/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph_utils.h"
#include "tagslam/logging.h"
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_utility.hpp>
#include <fstream>

namespace tagslam {
  namespace graph_utils {
    using std::string;
    class LabelWriter {
    public:
      LabelWriter(const Graph *g) : graph_(g)  { }
      template <class VertexOrEdge>
      void operator()(std::ostream &out, const VertexOrEdge& v) const {
        VertexConstPtr vp = graph_->getVertex(v);
        const string color =  graph_->isOptimized(v) ? "green" : "red";
        out << "[label=\"" << vp->getLabel() << "\", shape="
            << vp->getShape() << ", color=" << color << "]";
      }
    private:
      const Graph *graph_;
    };

    void plot(const string &fname, const Graph *g) {
      std::ofstream ofile(fname);
      boost::write_graphviz(ofile, g->getBoostGraph(), LabelWriter(g));
    }
    
    void plot_debug(const ros::Time &t, const string &tag, const Graph &g) {
      std::stringstream ss;
      ss << tag << "_" <<  t.toNSec() << ".dot";
      graph_utils::plot(ss.str(), &g);
    }



    static AbsolutePosePriorFactorPtr
    find_abs_pose_prior(const Graph &g, const VertexDesc &vv) {
      AbsolutePosePriorFactorPtr p;
      for (const auto &fv: g.getConnected(vv)) {
        p = std::dynamic_pointer_cast<factor::AbsolutePosePrior>(g[fv]);
        if (p) {
          break;
        }
      }
      return (p);
    }

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

    void add_tag(Graph *g, const Tag &tag) {
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

    void filter_graph(Graph *src,
                      const std::set<VertexDesc> &factorsToRemove,
                      const std::set<VertexDesc> &valuesToRemove) {
      Graph dest;
      std::set<VertexDesc> copiedValues;
      // first copy all values
      for (const auto &fac: src->getFactors()) {
        if (factorsToRemove.count(fac) == 0) { // 
          VertexVec values = src->getConnected(fac);
          for (const auto &v: values) {
            if (valuesToRemove.count(v) == 0 && copiedValues.count(v) == 0) {
              copiedValues.insert(v);
              // add value to dst graph
              GraphVertex destvp = src->getVertex(v)->clone();
              destvp->addToGraph(destvp, &dest); // add new value to graph
              // add initial values
              if (!src->isOptimized(v)) {
                BOMB_OUT("src value not optimized: " << src->info(v));
              }
              PoseValuePtr pdp= std::dynamic_pointer_cast<value::Pose>(destvp);
              if (!pdp) {
                BOMB_OUT("value is not a pose: " << src->info(v));
              }
              pdp->addToOptimizer(src->getOptimizedPose(v), &dest);
            }
          }
        }
      }
      // Then copy factors
      for (const auto &fac: src->getFactors()) {
        if (factorsToRemove.count(fac) == 0) { //
          FactorPtr fp =
            std::dynamic_pointer_cast<factor::Factor>(src->getVertex(fac));
          if (fp) {
            GraphVertex destfp = fp->clone();
            destfp->addToGraph(destfp, &dest); // will make connections also
          }
        }
      }
      *src = dest; // copy over source graph
    }
                      
    
    void copy_subgraph(Graph *dest, const Graph &src,
                       const std::deque<VertexDesc> &srcfacs,
                       double absPriorPositionNoise,
                       double absPriorRotationNoise) {
      // first copy the values
      std::set<VertexDesc> copiedVals; // track which values have been copied
      for (const auto &srcf: srcfacs) { // loop through factors
        ROS_DEBUG_STREAM(" copying for factor " << src.info(srcf));
        for (const auto &srcv: src.getConnected(srcf)) {
          if (copiedVals.count(srcv) == 0) { // avoid duplication
            copiedVals.insert(srcv);
            GraphVertex srcvp  = src.getVertex(srcv);
            GraphVertex destvp = srcvp->clone();
            destvp->addToGraph(destvp, dest); // add new value to graph
            AbsolutePosePriorFactorPtr pp = find_abs_pose_prior(src, srcv);
            if (pp) {
              // This pose is already pinned down by a pose prior.
              // Want to keep the flexibility specified in the config file!
              AbsolutePosePriorFactorPtr pp2 =
                std::dynamic_pointer_cast<factor::AbsolutePosePrior>(
                  pp->clone());
              pp2->addToGraph(pp2, dest);
            } else if (src.isOptimized(srcv)) {
              // Already established poses must be pinned down with a prior
              // If it's a camera pose, give it more flexibility
              PoseValuePtr srcpp =
                std::dynamic_pointer_cast<value::Pose>(srcvp);
              Transform pose = src.getOptimizedPose(srcv);
              const PoseNoise n =
                srcpp->isCameraPose() ? PoseNoise::make(0.05, 0.05) :
                PoseNoise::make(absPriorRotationNoise, absPriorPositionNoise);
              PoseWithNoise pwn(pose, n, true);
              AbsolutePosePriorFactorPtr
                pp(new factor::AbsolutePosePrior(destvp->getTime(), pwn,
                                                 destvp->getName()));
              // Add pose prior to graph
              pp->addToGraph(pp, dest);
            }
          }
        }
      }
      // now copy factors
      for (const auto &srcf: srcfacs) { // loop through factors
        FactorPtr fp =
          std::dynamic_pointer_cast<factor::Factor>(src.getVertex(srcf));
        if (fp) {
          GraphVertex destfp = fp->clone();
          destfp->addToGraph(destfp, dest);
        }
      }
    }
    
    void initialize_from(Graph *destg, const Graph &srcg) {
      // Look through the source graph for any values that are not yet
      // optimized in the destination graph. Initialize those values
      // in the destination graph, and add them to the optimizer.
      for (auto vi = srcg.getVertexIterator(); vi.first != vi.second;
           ++vi.first) {
        const VertexDesc sv = *vi.first;
        PoseValuePtr psp =
          std::dynamic_pointer_cast<value::Pose>(srcg[sv]);
        if (psp) {
          const VertexDesc dv = destg->find(psp->getId());
          if (!Graph::is_valid(dv)) {
            BOMB_OUT("cannot find dest value: " << psp->getLabel());
          }
          PoseValuePtr pdp =
            std::dynamic_pointer_cast<value::Pose>((*destg)[dv]);
          if (!pdp) {
            BOMB_OUT("invalid dest type: " << ((*destg)[dv])->getLabel());
          }
          if (!destg->isOptimized(dv) && srcg.isOptimized(sv)) {
            pdp->addToOptimizer(srcg.getOptimizedPose(sv), destg);
          }
        }
      }
      // now add all necessary factors to optimizer
      for (auto vi = srcg.getVertexIterator(); vi.first != vi.second;
           ++vi.first) {
        const VertexDesc sv = *vi.first;
        const FactorConstPtr sfp =
          std::dynamic_pointer_cast<factor::Factor>(srcg[sv]);
        if (sfp && !std::dynamic_pointer_cast<
            factor::AbsolutePosePrior>(srcg[sv])) {
          //ROS_DEBUG_STREAM("transferring factor: " << srcg.info(sv));
          VertexDesc dv = destg->find(sfp->getId());
          if (Graph::is_valid(dv)) {
            sfp->addToOptimizer(destg);
          } else {
            BOMB_OUT("no orig vertex found for: " << srcg.info(sv));
          }
        }
      }
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

    bool get_optimized_pose(const Graph &g, const Camera &cam,
                            Transform *tf) {
      return (get_optimized_pose(g, ros::Time(0),
                                 Graph::cam_name(cam.getName()), tf));
    }

    bool get_optimized_pose(const Graph &g, const ros::Time &t,
                            const Body &body, Transform *tf) {
      return (get_optimized_pose(g, t, Graph::body_name(body.getName()), tf));
    }

    bool get_optimized_pose(const Graph &g, const Tag &tag, Transform *tf) {
      return (get_optimized_pose(g, ros::Time(0),
                                 Graph::tag_name(tag.getId()), tf));
    }
    
    PoseWithNoise get_optimized_pose_with_noise(const Graph &g,
                                                const string &name) {
      PoseWithNoise pwn;
      const VertexDesc v = g.findPose(ros::Time(0), name);
      if (!Graph::is_valid(v) || !g.isOptimized(v)) {
        return (PoseWithNoise());
      }
      return (PoseWithNoise(g.getOptimizedPose(v), g.getPoseNoise(v), true));
    }
 

  }  // end of namespace
}  // end of namespace
