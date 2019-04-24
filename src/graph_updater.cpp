/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph_updater.h"
#include "tagslam/factor/tag_projection.h"
#include "tagslam/factor/relative_pose_prior.h"
#include "tagslam/pnp.h"
#include "tagslam/camera2.h"
#include "tagslam/tag2.h"

#include <boost/range/irange.hpp>
#include <boost/graph/graph_utility.hpp>
#include <cmath>
#include <algorithm>
#include <fstream>

namespace tagslam {

  using boost::irange;
  typedef GraphUpdater::VertexDeque VertexDeque;
  typedef GraphUpdater::VertexVec VertexVec;

  static bool contains(std::deque<VertexDesc> &c, const VertexDesc &v) {
    return (std::find(c.begin(), c.end(), v) != c.end());
  }
  static bool contains(std::vector<VertexDesc> &c, const VertexDesc &v) {
    return (std::find(c.begin(), c.end(), v) != c.end());
  }

  static int
  examine_connected_values(const Graph &graph, const VertexDesc &fac,
                           const SubGraph &covered, VertexDesc *valueVertex,
                           VertexDeque *values) {
    // find out if this factor allows us to determine a new value
    std::vector<VertexDesc> conn = graph.getConnected(fac);
    int numEdges(0), numValid(0);
    for (const auto vv: conn) {
      VertexConstPtr vvp = graph.getVertex(vv);
      ValueConstPtr   vp = std::dynamic_pointer_cast<const value::Value>(vvp);
      numEdges++;
      if ((vp && vp->isValid()) || covered.values.count(vv) != 0) {
        numValid++;
      } else {
        *valueVertex = vv;
      }
      values->push_back(vv);
    }
    return (numEdges - numValid);
  }

  void
  GraphUpdater::examine(const ros::Time &t, VertexDesc fac,
                        VertexDeque *factorsToExamine,
                        SubGraph *covered, SubGraph *newSubGraph) {
    ROS_DEBUG_STREAM("examining factor: " << graph_->info(fac));
    VertexDesc valueVertex;
    VertexDeque values;
    int numDetermined =
      examine_connected_values(*graph_, fac, *covered,
                               &valueVertex, &values);
    if (numDetermined == 1) {
      // establishes new value, let's explore the new
      VertexConstPtr vt = graph_->getVertex(valueVertex);
      ValueConstPtr vp = std::dynamic_pointer_cast<const value::Value>(vt);
      ROS_DEBUG_STREAM(" factor establishes new value: " << vp->getLabel());
      auto &ff = covered->factors;
      if (std::find(ff.begin(), ff.end(), fac) == ff.end()) {
        ff.push_back(fac);
        newSubGraph->factors.push_back(fac);
      }
      for (const auto vv: values) {
        newSubGraph->values.insert(vv);
        //ROS_INFO_STREAM("  adding new corresponding values: " << info(vv));
        covered->values.insert(vv);
      }
      VertexVec connFac = graph_->getConnected(valueVertex);
      for (const auto &fv: connFac) {
        VertexConstPtr  fvp = graph_->getVertex(fv); // pointer to factor
        FactorConstPtr fp =
          std::dynamic_pointer_cast<const factor::Factor>(fvp);
        if (fp) {
          if (fv != fac) { // no connections back
            ROS_DEBUG_STREAM("  " << vp->getLabel() << " activates "
                             << fp->getLabel());
            if (fp->getTime() == t || fp->getTime() == ros::Time(0)) {
              factorsToExamine->push_front(fv);
            } else {
              TimeToVertexesMap::iterator it = times_.find(fp->getTime());
              if (it == times_.end()) {
                ROS_DEBUG_STREAM("   first vertex at time: " << fp->getTime());
                it = times_.insert(
                  TimeToVertexesMap::value_type(fp->getTime(),
                                                VertexVec())).first;
              }
              auto &c = it->second;
              if (!contains(c, fv)) {
                ROS_DEBUG_STREAM("   remembering factor " << fp->getLabel());
                c.push_back(fv);
              } else {
                ROS_DEBUG_STREAM("   already remembered factor "
                                 << fp->getLabel());
              }
            }
          }
        }
      }
    } else if (numDetermined == 0) {
      // this factor does not establish a new value, but
      // provides an additional measurement on existing ones.
      ROS_DEBUG_STREAM(" factor provides additional measurement: "
                       << graph_->info(fac));
      auto &ff = covered->factors;
      if (std::find(ff.begin(), ff.end(), fac) == ff.end()) {
        ff.push_back(fac);
        newSubGraph->factors.push_back(fac);
      }
    } else {
      ROS_DEBUG_STREAM(" factor does not establish new values!");
    }
  }

  std::vector<VertexDeque>
  GraphUpdater::findSubgraphs(const ros::Time &t, const VertexVec &facs,
                              SubGraph *found) {
    profiler_.reset();
    std::vector<VertexDeque> sv;
    ROS_DEBUG_STREAM("===================== finding subgraphs for t = " << t);
    // first look over the new factors
    for (const auto &fac: facs) {
      ROS_DEBUG_STREAM(" ----- exploring new subgraph starting at: "
                       << graph_->info(fac));
      if (!contains(found->factors, fac)) {
        // this is a new factor that has not been explored
        SubGraph sg;
        exploreSubGraph(t, fac, &sg, found);
        if (!sg.factors.empty()) {
          sv.push_back(sg.factors);    // transfer factors
        }
      }
    }
    profiler_.record("findSubGraphs");
    return (sv);
  }

  double
  GraphUpdater::optimizeSubgraphs(const std::vector<GraphPtr> &subGraphs) {
    profiler_.reset();
    double totErr(0);
    for (auto &sg: subGraphs) {
      double err =  sg->optimizeFull();
      ROS_INFO_STREAM("error for subgraph optim: " << err);
      sg->transferOptimizedValues();
      totErr += err;
    }
    profiler_.record("optimizeSubgraphs");
    return (totErr);
  }

  double
  GraphUpdater::initializeFromSubgraphs(const std::vector<GraphPtr>
                                        &subGraphs) {
    profiler_.reset();
    double totalError(0);
    for (const auto &sg: subGraphs) {
      double err = sg->optimizeFull();
      double maxErr = sg->getMaxError();
      if (maxErr < maxSubgraphError_) {
        totalError += err;
        graph_->initializeFrom(*sg);
      } else { 
        ROS_WARN_STREAM("dropping subgraph with error: " << err << " "
                        << maxErr);
      }
    }
    profiler_.record("initialzeFromSubgraphs");
    return (totalError);
  }

  static int find_connected_poses(const Graph &graph,
                                  VertexDesc v,
                                  std::vector<PoseValuePtr> *poses,
                                  VertexVec *conn) {
    int missingIdx(-1), edgeNum(0), numMissing(0);
    *conn = graph.getConnected(v);
    poses->clear();
    for (const auto &vv : *conn) {
      VertexPtr   vvp = graph.getVertex(vv); // pointer to value
      PoseValuePtr pp = std::dynamic_pointer_cast<value::Pose>(vvp);
      if (!pp) {
        ROS_ERROR_STREAM("vertex is no pose: " << vv);
        throw std::runtime_error("vertex is no pose");
      }
      poses->push_back(pp);
      //ROS_DEBUG_STREAM(" factor attached value: " << pp->getLabel());
      if (!pp->isValid()) {
        missingIdx = edgeNum;
        numMissing++;
        //ROS_DEBUG_STREAM(" missing value: " << pp->getLabel());
      } else {
        //ROS_DEBUG_STREAM(" valid value: " << pp->getLabel());
      }
      edgeNum++;
    }
    missingIdx = (numMissing == 1) ? missingIdx : -(numMissing + 1);
    return (missingIdx);
  }

  static bool
  set_value_from_tag_projection(Graph *graph, VertexDesc v,
                                const Transform &T_c_o)  {
    std::vector<PoseValuePtr> T;
    VertexVec conn;
    int idx = find_connected_poses(*graph, v, &T, &conn);
    switch (idx) {
    case 0: { // T_r_c = T_r_w * T_w_b * T_b_o * T_o_c
      T[idx]->setPose(T[1]->getPose().inverse() * T[2]->getPose() *
                      T[3]->getPose() * T_c_o.inverse());
      T[idx]->addToOptimizer(graph);
      break; }
    case 1: { // T_w_r = T_w_b * T_b_o * T_o_c * T_c_r
      T[idx]->setPose(T[2]->getPose() * T[3]->getPose() *
                      T_c_o.inverse() * T[0]->getPose().inverse());
      T[idx]->addToOptimizer(graph);
      //ROS_DEBUG_STREAM("setting rig pose: " << T[idx]->getLabel() << std::endl << T[idx]->getPose());
      break; }
    case 2: { // T_w_b = T_w_r * T_r_c * T_c_o * T_o_b
      T[idx]->setPose(T[1]->getPose() * T[0]->getPose() *
                      T_c_o * T[3]->getPose().inverse());
      T[idx]->addToOptimizer(graph);
      break; }
    case 3: { // T_b_o = T_b_w * T_w_r * T_r_c * T_c_o
      T[idx]->setPose(T[2]->getPose().inverse() * T[1]->getPose() *
                      T[0]->getPose() * T_c_o);
      T[idx]->addToOptimizer(graph);
      break; }
    case -1: {
      // T_c_o = T_c_r[0] * T_r_w[1] * T_w_b[2] * T_b_o[3]
      Transform Test_c_o = T[0]->getPose().inverse() * T[1]->getPose().inverse() * T[2]->getPose() * T[3]->getPose();
      Transform poseDiff = T_c_o * Test_c_o.inverse();
      Eigen::AngleAxisd aa;
      aa.fromRotationMatrix(poseDiff.rotation());
      ROS_DEBUG_STREAM("dup factor with mismatch angle: " << aa.angle() << " len: " << poseDiff.translation().norm());
      return (true);
      break; }
    default: {
      ROS_DEBUG_STREAM("factor has multiple missing values: " << graph->getVertex(v)->getLabel());
      return (false);
      break; }
    }
    ROS_DEBUG_STREAM("tag proj setting value of " << T[idx]->getLabel());
    ROS_DEBUG_STREAM("set pose: " << std::endl << T[idx]->getPose());
    return (true);
  }

  static bool set_value_from_relative_pose_prior(
    Graph *graph, VertexDesc v, const Transform &deltaPose) {
    std::vector<PoseValuePtr> T;
    VertexVec conn;
    int idx = find_connected_poses(*graph, v, &T, &conn);
    if (T.size()  != 2) {
      ROS_ERROR_STREAM("rel pose prior has wrong num connected: " << T.size());
      throw std::runtime_error("rel pose prior has wrong num conn");
    }
    switch (idx) {
    case 0: { // T_0 = T_1 * delta T^-1
      T[idx]->setPose(T[1]->getPose() * deltaPose.inverse());
      T[idx]->addToOptimizer(graph);
      break; }
    case 1: { // T_1 = T_0 * delta T
      T[idx]->setPose(T[0]->getPose() * deltaPose);
      T[idx]->addToOptimizer(graph);
      break; }
    case -1: {
      ROS_DEBUG_STREAM("delta pose factor has no missing values!");
      return (true);
      break; }
    default: {
      ROS_DEBUG_STREAM("delta factor has multiple missing values!");
      return (false);
      break; }
    }
    ROS_DEBUG_STREAM("rel pos setting value of " << T[idx]->getLabel());
    ROS_DEBUG_STREAM("set pose: " << std::endl << T[idx]->getPose());
    return (true);
  }

  static void enumerate(std::vector<VertexDeque> *all,
                        const VertexDeque &prefix,
                        const VertexDeque &remain) {
    //all->push_back(remain);
    //return;
    int n = remain.size();
    for (int i = 0; i < n; i++) {
      VertexDeque p(remain.begin() + i, remain.end());
      p.insert(p.end(), remain.begin(), remain.begin() + i);
      all->push_back(p);
    }
    ROS_INFO_STREAM("made " << all->size() << " enumerations");
  }

  void GraphUpdater::setAngleLimit(double angDeg) {
    angleLimit_ = std::sin(angDeg / 180.0 * M_PI);
    
  }

  static bool initialize_subgraph(Graph *graph, double angleLimit,
                                  const VertexDeque &factors) {
    VertexDeque remainingFactors;
    // first see if we can already initialize via
    // any relative pose priors, since they are
    // generally more reliable
    for (const auto &v: factors) { //loop over factors
      RelativePosePriorFactorPtr rp =
        std::dynamic_pointer_cast<factor::RelativePosePrior>(graph->getVertex(v));
      if (rp && !rp->isOptimized() &&
          set_value_from_relative_pose_prior(graph, v, rp->getPoseWithNoise().getPose())) {
        rp->addToOptimizer(graph);
        ROS_DEBUG_STREAM("pre-init factor: " << graph->getVertex(v));
      } else {
        remainingFactors.push_back(v);
      }
    }
    
    // now do anything else
    for (const auto &v: remainingFactors) { //loop over factors
      VertexPtr  fvp = graph->getVertex(v);
      TagProjectionFactorPtr fp =
        std::dynamic_pointer_cast<factor::TagProjection>(fvp);
      if (fp) {
        // do homography for this vertex, add new values to graph and the optimizer!
        const CameraIntrinsics2 ci = fp->getCamera()->getIntrinsics();
        ROS_DEBUG_STREAM("computing pose for factor " << fvp->getLabel());
        auto tf = pnp::pose_from_4(fp->getImageCorners(),
                                   fp->getTag()->getObjectCorners(),
                                   ci.getK(), ci.getDistortionModel(), ci.getD());
        if (tf.second) {
          // viewing angle is determined by position of camera in tag
          // coordinates
          Eigen::Vector3d camPositionInObjFrame = tf.first.inverse().translation();
          camPositionInObjFrame.normalize();
          double sina = camPositionInObjFrame(2);
          if (sina > angleLimit) {
            double ang = std::asin(sina) / M_PI * 180;
            ROS_DEBUG_STREAM("got homography with angle " << ang << std::endl << tf.first);
            if (!set_value_from_tag_projection(graph, v, tf.first)) {
              return (false); // init failed!
            }
            if (!fp->isOptimized()) {
              // add factor and values to optimizer
              fp->setIsValid(true);
              fp->addToOptimizer(graph);
            }
          } else {
            ROS_DEBUG_STREAM("dropping factor " << graph->info(v) << " because of too small sin(ang): " << sina);
          }
        } else {
          ROS_WARN_STREAM("could not find valid homography!!");
        }
      } else {
        RelativePosePriorFactorPtr rp =
          std::dynamic_pointer_cast<factor::RelativePosePrior>(fvp);
        if (rp && !rp->isOptimized()) {
          // first fill in pose via deltaPose
          if (!set_value_from_relative_pose_prior(graph, v, rp->getPoseWithNoise().getPose())) {
            return (false); // init failed
          }
          // then add factor
          rp->addToOptimizer(graph);
        } else {
          ROS_DEBUG_STREAM("skipping factor: " << graph->info(v));
        }
      }
    }
    // test that all values have been covered
    bool allOptimized(true);
    for (const auto &fac: factors) {
      VertexVec values = graph->getConnected(fac);
      for (const auto &v: values) {
        if (!graph->getVertex(v)->isOptimized()) {
          allOptimized = false;
          break;
        }
      }
    }
    return (allOptimized);
  }

  void GraphUpdater::initializeSubgraphs(
    std::vector<GraphPtr> *subGraphs,
    const std::vector<VertexDeque> &verts) {
    profiler_.reset();
    ROS_DEBUG_STREAM("----------- initializing " << verts.size() << " subgraphs");
    subGraphs->clear();
    for (const auto &vs: verts) {  // iterate over all subgraphs
      ROS_DEBUG_STREAM("---------- subgraph of size: " << vs.size());
      std::vector<VertexDeque> orderings;
      enumerate(&orderings, VertexDeque(), vs);
      double errMin = 1e10;
      GraphPtr bestGraph;
      ROS_DEBUG_STREAM("number of orderings: " << orderings.size());
      int ord(0);
      for (const auto &factors: orderings) {
        ord++;
        GraphPtr sg(new Graph());
        Graph &subGraph = *sg;
        VertexDeque vset;
        // This makes a deep copy, hopefully
        subGraph.copyFrom(*graph_, factors, &vset);
        subGraph.print("init subgraph");
        if (initialize_subgraph(&subGraph, angleLimit_, vset)) {
          const auto errMap = subGraph.getErrorMap();
          double err =  subGraph.optimizeFull();
          double maxErr = subGraph.getMaxError();
          ROS_DEBUG_STREAM("ordering " << ord << " has error: " << err << " " << maxErr);
          if (maxErr >= 0 && maxErr < errMin) {
            errMin = maxErr;
            bestGraph = sg;
            ROS_DEBUG_STREAM("subgraph " << ord << " has minimum error: " << errMin);
            if (maxErr < maxSubgraphError_) {
              ROS_DEBUG_STREAM("breaking early due to low error");
              break;
            } else {
              ROS_DEBUG_STREAM("error map for subgraph: ");
              for (const auto &ev: errMap) {
                ROS_INFO_STREAM("SUBGRAPH ERROR_MAP  " << ev.first << " " << *(subGraph.getVertex(ev.second)));
              }
            }
          }
        } else {
          ROS_DEBUG_STREAM("subgraph rejected for ordering " << ord);
        }
      }
      if (bestGraph) {
        subGraphs->push_back(bestGraph);
        ROS_DEBUG_STREAM("best subgraph init found after " << ord << " attempts with error: " << errMin);
      } else {
        ROS_WARN_STREAM("could not initialize subgraph!");
      }
    }
    profiler_.record("initializeSubgraphs");
  }
#if 0
  void
  GraphUpdater::updateGraphFromFactors(Graph *g, const ros::Time &t,
                                       const VertexVec &facs,
                                       SubGraph *covered) {
    std::vector<VertexDeque> sv;
    sv = findSubgraphs(t, facs, covered);
    if (sv.empty()) {
      return;
    }
    std::vector<GraphPtr> subGraphs;
    
    initializeSubgraphs(&subGraphs, sv);
    optimizeSubgraphs(subGraphs);
    double serr = initializeFromSubgraphs(subGraphs);
    subgraphError_ += serr;
    
    double err = optimize(serr);
    
    ROS_INFO_STREAM("sum of subgraph err: " << subgraphError_ << ", full graph error: " << err);
    graph_->transferOptimizedValues();
  }
#endif

  double GraphUpdater::optimize(double thresh) {
    profiler_.reset();
    double error;
    if (optimizeFullGraph_) {
      error = graph_->optimizeFull();
      const auto errMap = graph_->getErrorMap();
      for (const auto &v: errMap) {
        if (v.first > 20.0) {
          ROS_INFO_STREAM("POSTOPT ERROR  " << v.first << " " << *(graph_->getVertex(v.second)));
        }
      }
    } else {
      if (numIncrementalOpt_ < maxNumIncrementalOpt_) {
        error = graph_->optimize(thresh);
        numIncrementalOpt_++;
      } else {
        ROS_INFO_STREAM("max count reached, running full optimization!");
        error = graph_->optimizeFull();
        graph_->transferFullOptimization();
        numIncrementalOpt_ = 0;
      }
    }
    profiler_.record("optimize");
    return (error);
  }

  void
  GraphUpdater::processNewFactors(const ros::Time &t,
                                  const VertexVec &facs) {
    ROS_DEBUG_STREAM("&&&&&&&&&&&&&&&&&&&&&&&&&&&&& got " << facs.size() << " new factors for t = " << t);
    if (facs.size() == 0) {
      ROS_DEBUG_STREAM("no new factors!");
      return;
    }
    SubGraph found;
    std::vector<VertexDeque> sv;
    sv = findSubgraphs(t, facs, &found);
    if (sv.empty()) {
      ROS_DEBUG_STREAM("no new factors activated!");
      return;
    }
    // ROS_DEBUG_STREAM("^^^^^^^^^^ checking complete graph for error before doing anything! ^^^^^^^^^^");
    // double err = graph_->getError();
    
    std::vector<GraphPtr> subGraphs;
    
    initializeSubgraphs(&subGraphs, sv);
    optimizeSubgraphs(subGraphs);
    double serr = initializeFromSubgraphs(subGraphs);
    subgraphError_ += serr;
    
    double err = optimize(serr);
    
    ROS_INFO_STREAM("sum of subgraph err: " << subgraphError_ << ", full graph error: " << err);
    graph_->transferOptimizedValues();
    
    std::cout << profiler_ << std::endl;
 
    ROS_DEBUG_STREAM("&-&-&-&-&-&-&-& done with new factors for t = " << t);
    while (!times_.empty()) {
      auto it = times_.rbegin();
      const ros::Time key = it->first;
      ROS_DEBUG_STREAM("++++++++++ handling old factors for t = " << key);
      sv = findSubgraphs(key, it->second, &found);
      initializeSubgraphs(&subGraphs, sv);
      optimizeSubgraphs(subGraphs);
      serr = initializeFromSubgraphs(subGraphs);
      subgraphError_ += serr;
      err = optimize(serr); // run global optimization
      ROS_INFO_STREAM("sum of subgraph err: " << subgraphError_ << ", full graph error: " << err);
      // now remove factors that have been used
      //ROS_DEBUG_STREAM("removing used factors...");
      for (auto ii = times_[key].begin(); ii != times_[key].end();) {
        if (contains(found.factors, *ii)) {
          //ROS_DEBUG_STREAM("removing used factor " << graph_->info(*ii) << " for time "  << key);
          ii = times_[key].erase(ii);
        } else {
          ++ii;
        }
      }
      if (times_[key].empty()) {
        ROS_DEBUG_STREAM("all elements gone for time " << key);
        times_.erase(key);
      }
      std::cout << profiler_ << std::endl;
      std::cout.flush();
      graph_->transferOptimizedValues();
    }
    ROS_INFO_STREAM("graph after update: " << graph_->getStats());
  }

  void
  GraphUpdater::exploreSubGraph(const ros::Time &t,
                         VertexDesc start,
                         SubGraph *subGraph, SubGraph *found) {
    VertexDeque factorsToExamine;
    factorsToExamine.push_back(start);
    while (!factorsToExamine.empty()) {
      VertexDesc exFac = factorsToExamine.front();
      factorsToExamine.pop_front();
      // examine() may append new factors to factorsToExamine
      examine(t, exFac, &factorsToExamine, found, subGraph);
    }
  }

}  // end of namespace
