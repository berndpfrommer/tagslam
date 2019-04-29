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

  static bool contains(const std::deque<VertexDesc> &c, const VertexDesc &v) {
    return (std::find(c.begin(), c.end(), v) != c.end());
  }
  static bool contains(const std::vector<VertexDesc> &c, const VertexDesc &v) {
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
      if ((vp && graph.isOptimized(vv)) || covered.values.count(vv) != 0) {
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
    //ROS_DEBUG_STREAM("examining factor: " << graph_->info(fac));
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
              TimeToVertexesMap::iterator it = oldFactors_.find(fp->getTime());
              if (it == oldFactors_.end()) {
                ROS_DEBUG_STREAM("   first vertex at time: " << fp->getTime());
                it = oldFactors_.insert(
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
      ROS_DEBUG_STREAM(" factor does not establish new values: " << graph_->info(fac));
    }
  }
  
  void
  GraphUpdater::exploreSubGraph(const ros::Time &t,
                         VertexDesc start,
                         SubGraph *subGraph, SubGraph *covered) {
    VertexDeque factorsToExamine;
    factorsToExamine.push_back(start);
    while (!factorsToExamine.empty()) {
      VertexDesc exFac = factorsToExamine.front();
      factorsToExamine.pop_front();
      // examine() may append new factors to factorsToExamine
      examine(t, exFac, &factorsToExamine, covered, subGraph);
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
  GraphUpdater::initializeFromSubgraphs(const std::vector<GraphPtr>
                                        &subGraphs) {
    profiler_.reset();
    double totalError(0);
    for (const auto &sg: subGraphs) {
      double err = sg->getError(); // assume already opt
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
                                  VertexDesc v, VertexVec *conn) {
    int missingIdx(-1), edgeNum(0), numMissing(0);
    *conn = graph.getConnected(v);
    for (const auto &vv : *conn) {
      VertexPtr   vvp = graph.getVertex(vv); // pointer to value
      PoseValuePtr pp = std::dynamic_pointer_cast<value::Pose>(vvp);
      if (!pp) {
        ROS_ERROR_STREAM("vertex is no pose: " << vv);
        throw std::runtime_error("vertex is no pose");
      }
      //ROS_DEBUG_STREAM(" factor attached value: " << pp->getLabel());
      if (!graph.isOptimized(vv)) {
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
  set_value_from_tag_projection(Graph *g, VertexDesc v,
                                const Transform &T_c_o)  {
    VertexVec T;
    int idx = find_connected_poses(*g, v, &T);
    Transform tf;
    switch (idx) {
    case 0: { // T_r_c = T_r_w * T_w_b * T_b_o * T_o_c
      tf =  g->pose(T[1]).inverse() * g->pose(T[2]) *
        g->pose(T[3]) * T_c_o.inverse();
      g->addToOptimizer(T[idx], tf);
      break; }
    case 1: { // T_w_r = T_w_b * T_b_o * T_o_c * T_c_r
      tf = g->pose(T[2]) * g->pose(T[3]) *
        T_c_o.inverse() * g->pose(T[0]).inverse();
      g->addToOptimizer(T[idx], tf);
      break; }
    case 2: { // T_w_b = T_w_r * T_r_c * T_c_o * T_o_b
      tf = g->pose(T[1]) * g->pose(T[0]) *
        T_c_o * g->pose(T[3]).inverse();
      g->addToOptimizer(T[idx], tf);
      break; }
    case 3: { // T_b_o = T_b_w * T_w_r * T_r_c * T_c_o
      tf = g->pose(T[2]).inverse() *g->pose(T[1]) *
        g->pose(T[0]) * T_c_o;
      g->addToOptimizer(T[idx], tf);
      break; }
    case -1: {
      // T_c_o = T_c_r[0] * T_r_w[1] * T_w_b[2] * T_b_o[3]
      const Transform Test_c_o = g->pose(T[0]).inverse() *
        g->pose(T[1]).inverse() * g->pose(T[2]) * g->pose(T[3]);
      const Transform poseDiff = T_c_o * Test_c_o.inverse();
      Eigen::AngleAxisd aa;
      aa.fromRotationMatrix(poseDiff.rotation());
      ROS_DEBUG_STREAM("dup factor with mismatch angle: " << aa.angle()
                       << " len: " << poseDiff.translation().norm());
      return (true);
      break; }
    default: {
      ROS_DEBUG_STREAM("factor has multiple missing values: "
                       << g->getVertex(v)->getLabel());
      return (false);
      break; }
    }
    ROS_DEBUG_STREAM("tag proj setting value of " << g->info(T[idx]));
    ROS_DEBUG_STREAM("set pose: " << std::endl << tf);
    return (true);
  }

  static bool set_value_from_relative_pose_prior(
    Graph *g, VertexDesc v, const Transform &deltaPose) {
    VertexVec T;
    int idx = find_connected_poses(*g, v, &T);
    if (T.size()  != 2) {
      ROS_ERROR_STREAM("rel pose prior has wrong num connected: " << T.size());
      throw std::runtime_error("rel pose prior has wrong num conn");
    }
    Transform tf;
    switch (idx) {
    case 0: { // T_0 = T_1 * delta T^-1
      tf = g->pose(T[1]) * deltaPose.inverse();
      g->addToOptimizer(T[idx], tf);
      break; }
    case 1: { // T_1 = T_0 * delta T
      tf = g->pose(T[0]) * deltaPose;
      g->addToOptimizer(T[idx], tf);
      break; }
    case -1: {
      ROS_DEBUG_STREAM("relative pose factor has no missing values!");
      return (true);
      break; }
    default: {
      ROS_DEBUG_STREAM("relative factor has multiple missing values!");
      return (false);
      break; }
    }
    ROS_DEBUG_STREAM("rel pos setting value of " << g->info(T[idx]));
    ROS_DEBUG_STREAM("set pose: " << std::endl << tf);
    return (true);
  }

  static void enumerate(std::vector<VertexDeque> *all,
                        const VertexDeque &orig) {
    // Simply rotate the factors through.
    // doing all permutations is prohibitively expensive
    int n = orig.size();
    for (int i = 0; i < n; i++) {
      VertexDeque p(orig.begin() + i, orig.end());
      p.insert(p.end(), orig.begin(), orig.begin() + i);
      all->push_back(p);
    }
  }

  void GraphUpdater::setMinimumViewingAngle(double angDeg) {
    minimumViewingAngle_ = angDeg;
  }

  static double viewing_angle(const Transform &tf) {
    // viewing angle is determined by position of camera in tag coord
    Eigen::Vector3d camPositionInObjFrame = tf.inverse().translation();
    camPositionInObjFrame.normalize();
    const double sina = camPositionInObjFrame(2); // z component = sin(angle);
    return (std::asin(sina) / M_PI * 180);
  }

  static bool init_from_abs_pose_prior(Graph *g, const VertexDesc &v) {
    AbsolutePosePriorFactorConstPtr ap =
      std::dynamic_pointer_cast<const factor::AbsolutePosePrior>((*g)[v]);
    if (ap && !g->isOptimized(v)) {
      VertexVec T;
      int idx = find_connected_poses(*g, v, &T);
      if (idx == 0) {
        ROS_DEBUG_STREAM("using abs pose prior: " << g->info(v));
        g->addToOptimizer(T[idx], ap->getPoseWithNoise().getPose());
        ap->addToOptimizer(g);
        return (true);
      }
    }
    return (false);
  }

  static bool init_from_rel_pose_prior(Graph *g, const VertexDesc &v) {
    RelativePosePriorFactorPtr rp =
      std::dynamic_pointer_cast<factor::RelativePosePrior>((*g)[v]);
    if (!rp || g->isOptimized(v)) {
      return (false);
    }
    if (set_value_from_relative_pose_prior(
          g, v, rp->getPoseWithNoise().getPose())) {
      ROS_DEBUG_STREAM("using rel pose prior: " << g->info(v));
      rp->addToOptimizer(g);
      return (true);
    } else {
      ROS_DEBUG_STREAM("cannot use rel pose prior: " << g->info(v));
    }
    return (false);
  }

  static bool init_from_proj_factor(Graph *g, const VertexDesc &v,
                                    double minAngle) {
    auto fp = std::dynamic_pointer_cast<factor::TagProjection>((*g)[v]);
    if (!fp) {
      return (false);
    }
    // do homography for this vertex
    const CameraIntrinsics2 ci = fp->getCamera()->getIntrinsics();
    ROS_DEBUG_STREAM("computing pose for factor " << g->info(v));
    auto rv = pnp::pose_from_4(fp->getImageCorners(),
                               fp->getTag()->getObjectCorners(),
                               ci.getK(), ci.getDistortionModel(), ci.getD());
    if (rv.second) { // got valid homography
      // viewing angle is determined by position of camera in tag coord
      const Transform &tf = rv.first;
      double ang = viewing_angle(tf);
      if (ang > minAngle) {
        ROS_DEBUG_STREAM("homography view angle: " << ang << std::endl << tf);
        if (set_value_from_tag_projection(g, v, tf)) {
          if (!g->isOptimized(v)) {
            // add factor to optimizer. The values are already there.
            fp->addToOptimizer(g);
          } else {
            ROS_DEBUG_STREAM("SHOULD NEVER HAPPEN: " << g->info(v));
            throw std::runtime_error("internal error");
            return (false);
          }
        } else {
          return (false); // not enough info to pin down pose!
        }
      } else {
        ROS_INFO_STREAM("drop " << g->info(v) << " small angle: " << ang);
        return (false);
      }
    } else {
      ROS_WARN_STREAM("could not find valid homography!!");
      return (false);
    }
    return (true);
  }

  static bool initialize_subgraph(Graph *g, double minViewAngle) {
    // first intialize poses from absolute pose priors since
    // those are the most reliable.
    VertexVec unhandled1;
    for (const auto &v: g->getFactors()) {
      if (init_from_abs_pose_prior(g, v)) {
        continue;
      }
      unhandled1.push_back(v);
    }
    // then initialize based on relative pose priors, which
    // are usually more reliable than projections
    VertexVec unhandled2;
    for (const auto &v: unhandled1) {
      if (!init_from_rel_pose_prior(g, v)) {
        unhandled2.push_back(v);
      }
    }
    // Now initialize based on projections *and* relative pose priors.
    // Relative pose priors must be considered again because for example
    // a projection factor may reveal the pose of a camera, which then will
    // allow the relative pose prior factor (to the extrinsic
    // camera calibration!) to become useful in determining the
    // camera-to-rig pose.
    for (const auto &v: unhandled2) {
      if (init_from_proj_factor(g, v, minViewAngle)) {
        continue;
      }
      if (init_from_rel_pose_prior(g, v)) {
        continue;
      }
    }
    // test that all values have been covered
    bool allOptimized(true);
    for (const auto &fac: g->getFactors()) {
      VertexVec values = g->getConnected(fac);
      for (const auto &v: values) {
        if (!g->isOptimized(v)) {
          allOptimized = false;
          ROS_DEBUG_STREAM("found unopt factor: " << g->info(v));
          break;
        }
      }
    }
    return (allOptimized);
  }

  static bool try_initialization(const Graph &g, const VertexDeque &factors,
                                 int ord, double minViewAngle, double errLimit,
                                 double *errMin, GraphPtr *bestGraph) {
    GraphPtr sg(new Graph());
    Graph &subGraph = *sg;
    // populate the subgraph with factors from the full graph
    subGraph.copyFrom(g, factors);
    //subGraph.print("init subgraph");
    if (initialize_subgraph(&subGraph, minViewAngle)) {
      double err    = subGraph.optimizeFull();
      double maxErr = subGraph.getMaxError();
      ROS_DEBUG_STREAM("ordering " << ord << " has error: " << err << " " << maxErr);
      if (maxErr >= 0 && maxErr < *errMin) {
        *errMin = maxErr;
        *bestGraph = sg;
        ROS_DEBUG_STREAM("subgraph " << ord << " has minimum error: " << *errMin);
        if (maxErr < errLimit) {
          //ROS_DEBUG_STREAM("breaking early due to low error");
          return (true);
        } else {
          ROS_DEBUG_STREAM("error map for subgraph: ");
          // move this statement earlier if you want to debug the
          // graph *before* it is optimized
          const auto errMap = subGraph.getErrorMap();
          for (const auto &ev: errMap) {
            ROS_INFO_STREAM("SUBGRAPH ERROR_MAP  " << ev.first
                            << " " << *(subGraph.getVertex(ev.second)));
          }
        }
      } 
    } else {
      ROS_DEBUG_STREAM("subgraph rejected for ordering " << ord);
    }
    return (false);
  }
  
  void
  GraphUpdater::initializeSubgraphs(std::vector<GraphPtr> *subGraphs,
                                    const std::vector<VertexDeque> &verts) {
    profiler_.reset();
    ROS_DEBUG_STREAM("------ initializing " << verts.size() << " subgraphs");
    subGraphs->clear();
    for (const auto &vs: verts) {  // iterate over all subgraphs
      ROS_DEBUG_STREAM("---------- subgraph of size: " << vs.size());
      std::vector<VertexDeque> orderings;
      enumerate(&orderings, vs);
      
      ROS_DEBUG_STREAM("number of factor orderings: " << orderings.size());
      GraphPtr bestGraph;
      int ord(0);
      double errMin = 1e10;
      for (const auto &ordering: orderings) {
        ord++;
        if (try_initialization(*graph_, ordering, ord, minimumViewingAngle_,
                               maxSubgraphError_, &errMin, &bestGraph)) {
          // found a good-enough error value!
          break;
        }
      }
      if (bestGraph) {  // found an acceptable error value
        subGraphs->push_back(bestGraph);
        ROS_DEBUG_STREAM("best subgraph init found after " << ord <<
                         " attempts with error: " << errMin);
      } else {
        ROS_WARN_STREAM("could not initialize subgraph!");
      }
    }
    profiler_.record("initializeSubgraphs");
  }

  double GraphUpdater::optimize(double thresh) {
    profiler_.reset();
    double error;
    if (optimizeFullGraph_) {
      error = graph_->optimizeFull();
    } else {
      if (numIncrementalOpt_ < maxNumIncrementalOpt_) {
#if 0        
        {
          const auto errMap = graph_->getErrorMap();
          int count(0);
          const int MAX_COUNT(1000000);
          for (auto it = errMap.rbegin(); it != errMap.rend() && count < MAX_COUNT; ++it, count++) {
            ROS_INFO_STREAM("ERROR_MAP_BEFORE  " << it->first
                            << " " << *((*graph_)[it->second]));
          }
        }
#endif        
        error = graph_->optimize(thresh);
        numIncrementalOpt_++;
#define REOPT        
#ifdef REOPT        
        // if there is a large increase in error, perform
        // a full optimization.
        // TODO: this is a terrible hack. Why does the
        // incremental optimizer fail? No idea.
        const double deltaErr = error - lastIncError_;
        if (deltaErr > 5 * thresh && deltaErr > 0.5 * (lastIncError_)) {
//#ifdef DEBUG
          const auto errMap = graph_->getErrorMap();
          int count(0);
          const int MAX_COUNT(1000000);
          for (auto it = errMap.rbegin(); it != errMap.rend() && count < MAX_COUNT; ++it, count++) {
            ROS_INFO_STREAM("ERROR_MAP  " << it->first
                            << " " << *((*graph_)[it->second]));
          }
//#endif          
          ROS_INFO_STREAM("large err increase: " << deltaErr << ", doing full optimization");
          error = graph_->optimizeFull(/*force*/ true);
          ROS_INFO_STREAM("error after full opt: " << error);
          graph_->transferFullOptimization();
          numIncrementalOpt_ = 0;
        }
        lastIncError_ = error;
#endif        
      } else {
        ROS_INFO_STREAM("max count reached, running full optimization!");
        error = graph_->optimizeFull(/*force*/ true);
        graph_->transferFullOptimization();
        numIncrementalOpt_ = 0;
      }
    }
    profiler_.record("optimize");
    return (error);
  }

  bool
  GraphUpdater::applyFactorsToGraph(const ros::Time &t, const VertexVec &facs,
                                    SubGraph *covered) {
    std::vector<VertexDeque> sv;
    sv = findSubgraphs(t, facs, covered);
    if (sv.empty()) {
      return (false);
    }
    std::vector<GraphPtr> subGraphs;
    initializeSubgraphs(&subGraphs, sv);
    //optimizeSubgraphs(subGraphs);
    double serr = initializeFromSubgraphs(subGraphs);
    subgraphError_ += serr;
    double err = optimize(serr);
    ROS_INFO_STREAM("sum of subgraph err: " << subgraphError_ <<
                    ", full graph error: " << err);
    eraseStoredFactors(t, covered->factors);
    return (true);
  }

  void
  GraphUpdater::processNewFactors(const ros::Time &t, const VertexVec &facs) {
    if (facs.empty()) {
      ROS_DEBUG_STREAM("no new factors received!");
      return;
    }
    // "covered" keeps track of what part of the graph has already
    // been operated on during this update cycle
    SubGraph covered;
    bool oldFactorsActivated = applyFactorsToGraph(t, facs, &covered);
    if (!oldFactorsActivated) {
      ROS_DEBUG_STREAM("no old factors activated!");
      return;
    }
    // The new measurements may have established previously
    // unknown poses (e.g. tag poses), thereby "activating"
    // factors that were useless before.
    while (!oldFactors_.empty()) {
      // work on the most recent factors
      const auto it = oldFactors_.rbegin();
      const ros::Time oldTime = it->first;
      ROS_DEBUG_STREAM("++++++++ handling " << it->second.size()
                       << " old factors for t = " << oldTime);
      if (!applyFactorsToGraph(oldTime, it->second, &covered)) {
        break;
      }
    }
    ROS_INFO_STREAM("graph after update: " << graph_->getStats());
  }

  void GraphUpdater::eraseStoredFactors(
    const ros::Time &t, const SubGraph::FactorCollection &covered) {
    profiler_.reset();
    const TimeToVertexesMap::iterator it = oldFactors_.find(t);
    if (it != oldFactors_.end()) {
      VertexVec &factors = it->second;
      // TODO: erasing individual elements from a vector
      // is inefficient. Use different structure
      for (auto ii = factors.begin(); ii != factors.end();) {
        if (contains(covered, *ii)) {
          //ROS_DEBUG_STREAM("removing used factor " <<
          //graph_->info(*ii) << " for time "  << key);
          ii = factors.erase(ii);
        } else {
          ++ii;
        }
      }
      if (factors.empty()) {
        oldFactors_.erase(t);
      }
    }
    profiler_.record("eraseStoredFactors");
  }

  void GraphUpdater::printPerformance() {
    ROS_INFO_STREAM("updater performance:");
    std::cout << profiler_ << std::endl;
  }

}  // end of namespace
