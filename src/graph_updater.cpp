/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph_updater.h"
#include "tagslam/factor/tag_projection.h"
#include "tagslam/factor/relative_pose_prior.h"
#include "tagslam/init_pose.h"
#include "tagslam/camera.h"
#include "tagslam/tag.h"
#include "tagslam/logging.h"
#include "tagslam/graph_utils.h"
#include "tagslam/xml.h"

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
    // returns number of missing values:
    //
    // 0: duplicate measurement
    // 1: one missing, may be able to establish a new pose!
    // >1: too many missing, out of luck
    //
    auto fp = factor::cast_const(graph[fac]);
    if (!fp) {
      BOMB_OUT("examined vertex is no factor: " + graph.info(fac));
    }
    // find out if this factor allows us to determine a new value
    std::vector<VertexDesc> conn = graph.getConnected(fac);
    int numEdges(0), numValid(0);
    for (const auto vv: conn) {
      VertexConstPtr vvp = graph.getVertex(vv);
      ValueConstPtr   vp = std::dynamic_pointer_cast<const value::Value>(vvp);
      numEdges++;
      if ((vp && graph.isOptimized(vv)) || covered.values.count(vv) != 0) {
        //ROS_DEBUG_STREAM("   pose is optimized: " << graph.info(vv));
        numValid++;
      } else {
        //ROS_DEBUG_STREAM("   pose is unoptimized: " << graph.info(vv));
        *valueVertex = vv;
      }
      values->push_back(vv);
    }
    int numMissing = numEdges - numValid;

    if (!fp->establishesValues() && numMissing == 1) {
      // if a factor cannot establish a full pose (e.g. a distance
      // measurement), then it can at best serve as a duplicate
      // measurement.
      numMissing = 2;
    }
    return (numMissing);
  }

  void GraphUpdater::setOptimizerMode(const std::string &mode) {
    if (mode == "FULL" || mode == "full") {
      optimizeFullGraph_ = true;
    }
  }

  void
  GraphUpdater::examine(Graph *graph, const ros::Time &t, VertexDesc fac,
                        VertexDeque *factorsToExamine,
                        SubGraph *covered, SubGraph *newSubGraph) {
    //ROS_DEBUG_STREAM("examining factor: " << graph->info(fac));
    VertexDesc valueVertex;
    VertexDeque values;
    int numMissing =
      examine_connected_values(*graph, fac, *covered,
                               &valueVertex, &values);
    if (numMissing == 1) {
      // establishes new value, let's explore the new
      VertexConstPtr vt = graph->getVertex(valueVertex);
      ValueConstPtr vp = std::dynamic_pointer_cast<const value::Value>(vt);
      ROS_DEBUG_STREAM(" factor " << graph->info(fac) << " establishes new value: " << vp->getLabel());
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
      VertexVec connFac = graph->getConnected(valueVertex);
      for (const auto &fv: connFac) {
        VertexConstPtr  fvp = graph->getVertex(fv); // pointer to factor
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
    } else if (numMissing == 0) {
      // this factor does not establish a new value, but
      // provides an additional measurement on existing ones.
      ROS_DEBUG_STREAM(" factor provides additional measurement: "
                       << graph->info(fac));
      auto &ff = covered->factors;
      if (std::find(ff.begin(), ff.end(), fac) == ff.end()) {
        ff.push_back(fac);
        newSubGraph->factors.push_back(fac);
        // Even though this factor might be a redundant (e.g. distance)
        // measurement, it could be that some of its connected values
        // are optimized, but not pinned down by any other factor.
        // Therefore we add those values here. When the values are
        // later copied, an absolute pose prior will be automatically
        // added to them.
        for (const auto vv: values) {
          newSubGraph->values.insert(vv);
          covered->values.insert(vv);
        }
      }
    } else {
      ROS_DEBUG_STREAM(" factor does not establish new values: " << graph->info(fac));
    }
  }
  
  void
  GraphUpdater::exploreSubGraph(Graph *graph, const ros::Time &t,
                                VertexDesc start,
                                SubGraph *subGraph, SubGraph *covered) {
    VertexDeque factorsToExamine;
    factorsToExamine.push_back(start);
    while (!factorsToExamine.empty()) {
      VertexDesc exFac = factorsToExamine.front();
      factorsToExamine.pop_front();
      // examine() may append new factors to factorsToExamine
      examine(graph, t, exFac, &factorsToExamine, covered, subGraph);
    }
  }


  std::vector<VertexDeque>
  GraphUpdater::findSubgraphs(Graph *graph, const ros::Time &t,
                              const VertexVec &facs, SubGraph *found) {
    profiler_.reset("findSubGraphs");
    std::vector<VertexDeque> sv;
    ROS_DEBUG_STREAM("===================== finding subgraphs for t = " << t);
    // first look over the new factors
    for (const auto &fac: facs) {
      ROS_DEBUG_STREAM(" ----- exploring new subgraph starting at: "
                       << graph->info(fac));
      if (!contains(found->factors, fac)) {
        // this is a new factor that has not been explored
        SubGraph sg;
        exploreSubGraph(graph, t, fac, &sg, found);
        if (!sg.factors.empty()) {
          sv.push_back(sg.factors);    // transfer factors
        }
      }
    }
    profiler_.record("findSubGraphs");
    return (sv);
  }

  static int find_connected_poses(const Graph &graph,
                                  VertexDesc v, VertexVec *conn) {
    int missingIdx(-1), edgeNum(0), numMissing(0);
    *conn = graph.getConnected(v);
    for (const auto &vv : *conn) {
      VertexPtr   vvp = graph.getVertex(vv); // pointer to value
      PoseValuePtr pp = std::dynamic_pointer_cast<value::Pose>(vvp);
      if (!pp) {
        BOMB_OUT("vertex is no pose: " << vv);
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
      g->getPoseVertex(T[idx])->addToOptimizer(tf, g);
      break; }
    case 1: { // T_w_r = T_w_b * T_b_o * T_o_c * T_c_r
      tf = g->pose(T[2]) * g->pose(T[3]) *
        T_c_o.inverse() * g->pose(T[0]).inverse();
      g->getPoseVertex(T[idx])->addToOptimizer(tf, g);
      break; }
    case 2: { // T_w_b = T_w_r * T_r_c * T_c_o * T_o_b
      tf = g->pose(T[1]) * g->pose(T[0]) *
        T_c_o * g->pose(T[3]).inverse();
      g->getPoseVertex(T[idx])->addToOptimizer(tf, g);
      break; }
    case 3: { // T_b_o = T_b_w * T_w_r * T_r_c * T_c_o
      tf = g->pose(T[2]).inverse() *g->pose(T[1]) *
        g->pose(T[0]) * T_c_o;
      g->getPoseVertex(T[idx])->addToOptimizer(tf, g);
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
      BOMB_OUT("rel pose prior has wrong num connected: " << T.size());
    }
    Transform tf;
    switch (idx) {
    case 0: { // T_0 = T_1 * delta T^-1
      tf = g->pose(T[1]) * deltaPose.inverse();
      g->getPoseVertex(T[idx])->addToOptimizer(tf, g);
      break; }
    case 1: { // T_1 = T_0 * delta T
      tf = g->pose(T[0]) * deltaPose;
      g->getPoseVertex(T[idx])->addToOptimizer(tf, g);
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

  static bool init_from_abs_pose_prior(Graph *g, const VertexDesc &v) {
    AbsolutePosePriorFactorConstPtr ap =
      std::dynamic_pointer_cast<const factor::AbsolutePosePrior>((*g)[v]);
    if (ap && !g->isOptimized(v)) {
      VertexVec T;
      int idx = find_connected_poses(*g, v, &T);
      if (idx == 0) {
        ROS_DEBUG_STREAM("using abs pose prior: " << g->info(v));
        g->getPoseVertex(T[idx])->addToOptimizer(
          ap->getPoseWithNoise().getPose(), g);
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
                                    const init_pose::Params &poseInitParams) {
    auto fp = std::dynamic_pointer_cast<factor::TagProjection>((*g)[v]);
    if (!fp) {
      return (false);
    }
    // do homography for this vertex
    const CameraIntrinsics ci = fp->getCamera()->getIntrinsics();
    ROS_DEBUG_STREAM("computing pose for factor " << g->info(v));
    auto rv = init_pose::pose_from_4(
      fp->getImageCorners(), fp->getTag()->getObjectCorners(),
      ci.getK(), ci.getDistortionModel(), ci.getD(), poseInitParams);
    if (rv.second) { // got valid homography
      const Transform &tf = rv.first;
      if (set_value_from_tag_projection(g, v, tf)) {
        if (!g->isOptimized(v)) {
          // add factor to optimizer. The values are already there.
          fp->addToOptimizer(g);
        } else {
          BOMB_OUT("SHOULD NEVER HAPPEN: " << g->info(v));
          return (false);
        }
      } else {
        return (false); // not enough info to pin down pose!
      }
    } else {
      ROS_INFO_STREAM("could not find valid homography!");
      return (false);
    }
    return (true);
  }

  static std::pair<std::set<VertexDesc>,std::set<VertexDesc>>
  find_vertexes_to_remove(const Graph &g) {
    std::pair<std::set<VertexDesc>, std::set<VertexDesc>> vertexesToRemove;
    for (const auto &fac: g.getFactors()) {
      VertexVec values = g.getConnected(fac);
      for (const auto &v: values) {
        if (!g.isOptimized(v)) {
          ROS_DEBUG_STREAM("found unopt factor: " << g.info(v));
          vertexesToRemove.second.insert(v); // must remove this value
          if (g.getConnected(v).size() <= 1) {
            // only connected to the network by this factor, safe to remove!
            vertexesToRemove.first.insert(fac); // can remove this factor
          }
        }
      }
    }
    return (vertexesToRemove);
  }

  static bool initialize_subgraph(Graph *g, const init_pose::Params &params) {
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
      if (init_from_proj_factor(g, v, params)) {
        continue;
      }
      if (init_from_rel_pose_prior(g, v)) {
        continue;
      }
    }
    // remove from subgraph any tag poses (and only those!) that
    // could not be reliably determined
    auto toRemove = find_vertexes_to_remove(*g);
    bool initSuccessful = toRemove.second.empty();
    if (!initSuccessful && (toRemove.first.size() == toRemove.second.size())) {
      // every uninit value has exactly one factor connected, can filter
      graph_utils::filter_graph(g, toRemove.first, toRemove.second);
      initSuccessful = true;
    }
    return (initSuccessful);
  }

  static bool try_initialization(const Graph &g, const VertexDeque &factors,
                                 int ord,
                                 const init_pose::Params &poseInitParams,
                                 double errLimit,
                                 double absPriorPositionNoise,
                                 double absPriorRotationNoise,
                                 double *errMin, GraphPtr *bestGraph,
                                 Profiler *profiler) {
    GraphPtr sg(new Graph());
    Graph &subGraph = *sg;
    // populate the subgraph with factors from the full graph
    graph_utils::copy_subgraph(sg.get(), g, factors, absPriorPositionNoise,
                               absPriorRotationNoise);
    //subGraph.print("init subgraph");
    if (initialize_subgraph(&subGraph, poseInitParams)) {
      profiler->reset("subgraphOpt");
      double err    = subGraph.optimizeFull();
      profiler->record("subgraphOpt");
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
  
  double
  GraphUpdater::initializeSubgraphs(Graph *graph,
                                    std::vector<GraphPtr> *subGraphs,
                                    const std::vector<VertexDeque> &verts) {
    double totalSGError(0);

    ROS_DEBUG_STREAM("------ initializing " << verts.size() << " subgraphs");
    subGraphs->clear();
    for (const auto &vs: verts) {  // iterate over all subgraphs
      ROS_DEBUG_STREAM("---------- subgraph of size: " << vs.size());
      profiler_.reset("initializeSubgraphs");
      std::vector<VertexDeque> orderings;
      enumerate(&orderings, vs);
      
      ROS_DEBUG_STREAM("number of factor orderings: " << orderings.size());
      GraphPtr bestGraph;
      int ord(0);
      double errMin = 1e10;
      for (const auto &ordering: orderings) {
        ord++;
        if (try_initialization(*graph, ordering, ord, poseInitParams_,
                               maxSubgraphError_,
                               subGraphAbsPriorPositionNoise_,
                               subGraphAbsPriorRotationNoise_,
                               &errMin, &bestGraph, &profiler_)) {
          // found a good-enough error value
          break;
        }
      }
      profiler_.record("initializeSubgraphs");
      if (bestGraph) {  // found an acceptable error value
        profiler_.reset("initializeFromSubgraphs");
        subGraphs->push_back(bestGraph);
        ROS_DEBUG_STREAM("best subgraph init found after " << ord <<
                         " attempts with error: " << errMin);
        const double sgErr = bestGraph->getError();
        const double maxErr = bestGraph->getMaxError();
        if (maxErr < maxSubgraphError_) {
          totalSGError += sgErr;
          graph_utils::initialize_from(graph, *bestGraph);
        } else { 
          ROS_WARN_STREAM("dropping subgraph with error: " << sgErr << " "
                          << maxErr);
        }
        //bestGraph->printErrorMap("BEST SUBGRAPH");
        profiler_.record("initializeFromSubgraphs");
      } else {
        ROS_WARN_STREAM("could not initialize subgraph!");
      }
    }
    return (totalSGError);
  }

  double GraphUpdater::optimize(Graph *graph, double thresh) {
    profiler_.reset("optimize");
    double error;
    if (optimizeFullGraph_) {
      error = graph->optimizeFull();
    } else {
      if (numIncrementalOpt_ < maxNumIncrementalOpt_) {
#ifdef DEBUG
        {
          const auto errMap = graph->getErrorMap();
          int count(0);
          const int MAX_COUNT(1000000);
          for (auto it = errMap.rbegin(); it != errMap.rend() && count < MAX_COUNT; ++it, count++) {
            ROS_INFO_STREAM("ERROR_MAP_BEFORE  " << it->first
                            << " " << *((*graph)[it->second]));
          }
          //graph->printErrorMap("ERROR DETAILS BEFORE");
        }
#endif        
        error = graph->optimize(thresh);
        numIncrementalOpt_++;
        // if there is a large increase in error, perform
        // a full optimization.
        // TODO: this is a terrible hack. Why does the
        // incremental optimizer fail? No idea.
        const double deltaErr = error - lastIncError_;
        const double MIN_DELTA_ERR = 1.0;
        if (deltaErr > 5 * thresh &&
            deltaErr > MIN_DELTA_ERR) {
          ROS_INFO_STREAM("large err inc: " << deltaErr << " vs " << thresh <<
                          ", doing full optimization");
          error = graph->optimizeFull(/*force*/ true);
          ROS_INFO_STREAM("error after full opt: " << error);
          graph->transferFullOptimization();
          numIncrementalOpt_ = 0;
        }
        lastIncError_ = error;
      } else {
        ROS_INFO_STREAM("max count reached, running full optimization!");
        error = graph->optimizeFull(/*force*/ true);
        graph->transferFullOptimization();
        numIncrementalOpt_ = 0;
      }
    }
    profiler_.record("optimize");
    return (error);
  }

  bool
  GraphUpdater::applyFactorsToGraph(Graph *graph, const ros::Time &t,
                                    const VertexVec &facs, SubGraph *covered) {
    std::vector<VertexDeque> sv;
    sv = findSubgraphs(graph, t, facs, covered);
    if (sv.empty()) {
      return (false);
    }
    std::vector<GraphPtr> subGraphs;
    const double serr = initializeSubgraphs(graph, &subGraphs, sv);
    subgraphError_ += serr;
    const double err = optimize(graph, serr);
    if (err >= 0) {
      ROS_INFO_STREAM("sum of subgraph err: " << subgraphError_ <<
                      ", full graph error: " << err);
    } else {
      ROS_INFO_STREAM("sum of subgraph err: " << subgraphError_);
    }
    eraseStoredFactors(t, covered->factors);
    return (true);
  }

  void
  GraphUpdater::processNewFactors(Graph *graph, const ros::Time &t,
                                  const VertexVec &facs) {
    profiler_.reset("processNewFactors");
    if (facs.empty()) {
      ROS_DEBUG_STREAM("no new factors received!");
      profiler_.record("processNewFactors");
      return;
    }
    // "covered" keeps track of what part of the graph has already
    // been operated on during this update cycle
    SubGraph covered;
    bool oldFactorsActivated = applyFactorsToGraph(graph, t, facs, &covered);
    if (!oldFactorsActivated) {
      ROS_DEBUG_STREAM("no old factors activated!");
      profiler_.record("processNewFactors");
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
      if (!applyFactorsToGraph(graph, oldTime, it->second, &covered)) {
        break;
      }
    }
    ROS_INFO_STREAM("graph after update: " << graph->getStats());
    profiler_.record("processNewFactors");
  }

  void GraphUpdater::eraseStoredFactors(
    const ros::Time &t, const SubGraph::FactorCollection &covered) {
    profiler_.reset("eraseStoredFactors");
    const TimeToVertexesMap::iterator it = oldFactors_.find(t);
    if (it != oldFactors_.end()) {
      VertexVec &factors = it->second;
      // TODO: erasing individual elements from a vector
      // is inefficient. Use different structure
      for (auto ii = factors.begin(); ii != factors.end();) {
        if (contains(covered, *ii)) {
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

  void GraphUpdater::parse(XmlRpc::XmlRpcValue config) {
    if (!config.hasMember("tagslam_parameters")) {
      BOMB_OUT("tagslam config file must have tagslam_parameters!");
    }
    XmlRpc::XmlRpcValue cfg = config["tagslam_parameters"];
    try {
      pixelNoise_ = xml::parse<double>(cfg, "pixel_noise", 1.0);
      poseInitParams_.minViewingAngle =
        xml::parse<double>(cfg, "minimum_viewing_angle", 20.0);
      poseInitParams_.ambiguityAngleThreshold =
        xml::parse<double>(cfg, "ambiguity_angle_threshold", 60.0)/180.0*M_PI;
      poseInitParams_.maxAmbiguityRatio =
        xml::parse<double>(cfg, "max_ambiguity_ratio", 0.3);
      maxSubgraphError_ = xml::parse<double>(cfg, "max_subgraph_error", 50.0);
      subGraphAbsPriorPositionNoise_ =
        xml::parse<double>(cfg, "subgraph_abs_prior_position_noise", 0.001);
      subGraphAbsPriorRotationNoise_ =
        xml::parse<double>(cfg, "subgraph_abs_prior_rotation_noise", 0.001);
      maxNumIncrementalOpt_ =
        xml::parse<int>(cfg, "max_num_incremental_opt", 100);
      optimizerMode_ = xml::parse<string>(cfg, "optimizer_mode", "slow");
      setOptimizerMode(optimizerMode_);
    } catch (const XmlRpc::XmlRpcException &e) {
      BOMB_OUT("error parsing tagslam_parameters!");
    }
  }

}  // end of namespace
