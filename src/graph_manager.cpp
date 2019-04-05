/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/graph_manager.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/factor/absolute_pose_prior.h"
#include "tagslam/factor/relative_pose_prior.h"
#include "tagslam/factor/tag_projection.h"
#include "tagslam/pose_with_noise.h"
#include "tagslam/value/pose.h"
#include "tagslam/optimizer.h"
#include "tagslam/pnp.h"

#include <boost/range/irange.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_utility.hpp>
#include <vector>
#include <algorithm>
#include <fstream>
#include <queue>
#include <map>

//#define DEBUG_GRAPH

namespace tagslam {

  using boost::irange;

  static bool contains(std::deque<VertexDesc> &c,
                       const VertexDesc &v) {
    return (std::find(c.begin(), c.end(), v) != c.end());
  }
  static bool contains(std::vector<VertexDesc> &c,
                       const VertexDesc &v) {
    return (std::find(c.begin(), c.end(), v) != c.end());
  }
  
  GraphManager::GraphManager() {
    graph_.setVerbosity("TERMINATION");
  }

  GraphManager::~GraphManager() {
    std::cout << profiler_ << std::endl;
    std::cout.flush();
  }

  double GraphManager::optimize() {
    profiler_.reset();
    double error;
    if (optimizeFullGraph_) {
      error = graph_.optimizeFull();
    } else {
      if (numIncrementalOpt_ < maxNumIncrementalOpt_) {
        error = graph_.optimize();
        numIncrementalOpt_++;
      } else {
        ROS_INFO_STREAM("max count reached, running full optimization!");
        error = graph_.optimizeFull();
        graph_.transferFullOptimization();
        numIncrementalOpt_ = 0;
      }
    }
    profiler_.record("optimize");
    return (error);
  }

  void GraphManager::addBody(const Body &body) {
    // add body pose as vertex
    if (body.isStatic()) {
      const ros::Time t0(0);
      if (body.getPoseWithNoise().isValid()) {
        const PoseWithNoise &pn = body.getPoseWithNoise();
        string name = Graph::body_name(body.getName());
        addPoseWithPrior(t0, name, pn);
      }
    } 
    // add associated tags as vertices
    for (const auto &tag: body.getTags()) {
      addTag(*tag);
    }
    ROS_INFO_STREAM("added body " << body.getName() << " with "
                    << body.getTags().size() << " tags");
  }

  void
  GraphManager::addTag(const Tag2 &tag) {
    const string name = Graph::tag_name(tag.getId());
    const ros::Time t0(0);
    if (tag.getPoseWithNoise().isValid()) {
      addPoseWithPrior(t0, name, tag.getPoseWithNoise());
    } else {
      graph_.addPose(t0, name, tag.getPoseWithNoise().getPose(), false);
    }
  }

  bool
  GraphManager::getPose(const ros::Time &t, const string &name,
                        Transform *tf) const {
    VertexDesc v = graph_.findPose(t, name);
    if (!Graph::is_valid(v) || !graph_.isOptimized(v)) {
      return (false);
    }
    *tf = graph_.getOptimizedPose(v);
    return (true);
  }

  PoseWithNoise
  GraphManager::getCameraPoseWithNoise(const Camera2ConstPtr &cam) const {
    PoseWithNoise pwn;
    VertexDesc v = graph_.findPose(ros::Time(0), Graph::cam_name(cam->getName()));
    if (!Graph::is_valid(v) || !graph_.isOptimized(v)) {
      return (PoseWithNoise());
    }
    return (PoseWithNoise(graph_.getOptimizedPose(v), graph_.getPoseNoise(v), true));
  }
  
  VertexDesc
  GraphManager::addPrior(const ros::Time &t, const string &name,
                         const PoseWithNoise &pn) {
    std::shared_ptr<factor::AbsolutePosePrior>
      fac(new factor::AbsolutePosePrior(t, pn, name));
    VertexDesc v = graph_.add(fac);
    fac->addToOptimizer(&graph_);
    return (v);
  }
  
  VertexDesc
  GraphManager::addProjectionFactor(const ros::Time &t,
                                    const Tag2ConstPtr &tag,
                                    const Camera2ConstPtr &cam,
                                    const geometry_msgs::Point *imgCorners) {
    TagProjectionFactorPtr fac(
      new factor::TagProjection(t, cam, tag, imgCorners, pixelNoise_,
                                cam->getName() + "-" + Graph::tag_name(tag->getId())));

    return (graph_.add(fac));
  }
  
  VertexDesc
  GraphManager::addPoseWithPrior(const ros::Time &t, const string &name,
                                 const PoseWithNoise &pn) {
    VertexDesc v = graph_.addPose(t, name, pn.getPose(), true);
    graph_.getVertex(v)->addToOptimizer(&graph_);
    VertexDesc pv = addPrior(t, name, pn);
    return (pv);
  }

  VertexDesc
  GraphManager::addPose(const ros::Time &t, const string &name,
                        const Transform &pose, bool poseIsValid) {
    if (graph_.hasPose(t, name)) {
      ROS_ERROR_STREAM("duplicate pose added, id: " << t << " " << name);
      throw std::runtime_error("duplicate pose added!");
    }
    VertexDesc npv = graph_.addPose(t, name, pose, poseIsValid);
    return (npv);
  }

  VertexDesc
  GraphManager::addBodyPoseDelta(const ros::Time &tPrev, const ros::Time &tCurr,
                                 const BodyConstPtr &body,
                                 const PoseWithNoise &deltaPose) {
    Transform prevPose;
    string      name = Graph::body_name(body->getName());
    VertexDesc pp = graph_.findPose(tPrev, name);
    VertexDesc cp = graph_.findPose(tCurr, name);
    
    if (!Graph::is_valid(pp)) {
      ROS_DEBUG_STREAM("adding previous pose for " << name << " " << tPrev);
      pp = addPose(tPrev, name, Transform::Identity(), false);
    }
    if (!Graph::is_valid(cp)) {
      ROS_DEBUG_STREAM("adding current pose for " << name << " " << tCurr);
      cp = addPose(tCurr, name, Transform::Identity(), false);
    }
    RelativePosePriorFactorPtr fac(new factor::RelativePosePrior(tCurr, tPrev, deltaPose, name));
    return (graph_.add(fac));
  }

  void
  GraphManager::examine(const ros::Time &t, VertexDesc fac,
                 std::deque<VertexDesc> *factorsToExamine,
                 SubGraph *found, SubGraph *sg) {
    ROS_DEBUG_STREAM("examining factor: " << graph_.info(fac));
    // find out if this factor allows us to determine a new value
    std::vector<VertexDesc> conn = graph_.getConnected(fac);
    int numEdges(0), numValid(0);
    VertexDesc valueVertex;
    std::deque<VertexDesc> values;
    for (const auto vv: conn) {
      VertexConstPtr vvp = graph_.getVertex(vv);
      ValueConstPtr   vp = std::dynamic_pointer_cast<const value::Value>(vvp);
      numEdges++;
      if ((vp && vp->isValid()) || found->values.count(vv) != 0) {
        //ROS_INFO_STREAM(" has valid    value: " << vp->getLabel());
        numValid++;
      } else {
        //ROS_INFO_STREAM(" has no valid value: " << vp->getLabel());
        valueVertex = vv;
      }
      values.push_back(vv);
    }
    if (numValid == numEdges - 1) {
      // establishes new value, let's explore the new
      VertexConstPtr vt = graph_.getVertex(valueVertex);
      ValueConstPtr vp = std::dynamic_pointer_cast<const value::Value>(vt);
      ROS_DEBUG_STREAM(" factor establishes new value: " << vp->getLabel());
      auto &ff = found->factors;
      if (std::find(ff.begin(), ff.end(), fac) == ff.end()) {
        ff.push_back(fac);
        sg->factors.push_back(fac);
      }
      for (const auto vv: values) {
        sg->values.insert(vv);
        //ROS_INFO_STREAM("  adding new corresponding values: " << info(vv));
        found->values.insert(vv);
      }
      std::vector<VertexDesc> connFac = graph_.getConnected(valueVertex);
      for (const auto &fv: connFac) {
        VertexConstPtr  fvp = graph_.getVertex(fv); // pointer to factor
        FactorConstPtr   fp = std::dynamic_pointer_cast<const factor::Factor>(fvp);
        if (fp) {
          if (fv != fac) { // no connections back
            ROS_DEBUG_STREAM("  " << vp->getLabel() << " activates " << fp->getLabel());
            if (fp->getTime() == t || fp->getTime() == ros::Time(0)) {
              factorsToExamine->push_front(fv);
            } else {
              TimeToVertexesMap::iterator it = times_.find(fp->getTime());
              if (it == times_.end()) {
                ROS_DEBUG_STREAM("   first vertex at time: " << fp->getTime());
                it = times_.insert(TimeToVertexesMap::value_type(fp->getTime(), std::vector<VertexDesc>())).first;
              }
              auto &c = it->second;
              if (!contains(c, fv)) {
                ROS_DEBUG_STREAM("   remembering factor " << fp->getLabel());
                c.push_back(fv);
              } else {
                ROS_DEBUG_STREAM("   already remembered factor " << fp->getLabel());
              }
            }
          }
        }
      }
    } else if (numValid == numEdges) {
      // this factor does not establish a new value, but
      // provides an additional measurement on existing ones.
      ROS_DEBUG_STREAM(" factor provides additional measurement: " << graph_.info(fac));
      auto &ff = found->factors;
      if (std::find(ff.begin(), ff.end(), fac) == ff.end()) {
        ff.push_back(fac);
        sg->factors.push_back(fac);
      }
    } else {
      ROS_DEBUG_STREAM(" factor does not establish new values!");
    }
  }

  std::vector<std::deque<VertexDesc>>
  GraphManager::findSubgraphs(const ros::Time &t,
                       const std::vector<VertexDesc> &facs,
                       SubGraph *found) {
    profiler_.reset();
    std::vector<std::deque<VertexDesc>> sv;
    ROS_DEBUG_STREAM("======================= finding subgraphs for t = " << t);
    // first look over the new factors
    for (const auto &fac: facs) {
      ROS_DEBUG_STREAM(" ----- exploring new subgraph starting at: " << graph_.info(fac));
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
  GraphManager::optimizeSubgraphs(const std::vector<GraphPtr> &subGraphs) {
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

  void
  GraphManager::initializeFromSubgraphs(const std::vector<GraphPtr> &subGraphs) {
    profiler_.reset();
    for (const auto &sg: subGraphs) {
      double err = sg->optimizeFull();
      double maxErr = sg->getMaxError();
      if (maxErr < maxSubgraphError_) {
        graph_.initializeFrom(*sg);
      } else { 
        ROS_WARN_STREAM("dropping subgraph with error: " << err << " " << maxErr);
      }
    }
    profiler_.record("initialzeFromSubgraphs");
  }

  static int find_connected_poses(const Graph &graph,
                                  VertexDesc v,
                                  std::vector<PoseValuePtr> *poses,
                                  std::vector<VertexDesc> *conn) {
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

  static bool set_value_from_tag_projection(Graph *graph,
                                            VertexDesc v, const Transform &T_c_o)  {
    std::vector<PoseValuePtr> T;
    std::vector<VertexDesc> conn;
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
    std::vector<VertexDesc> conn;
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

  static void enumerate(std::vector<std::deque<VertexDesc>> *all,
                        const std::deque<VertexDesc> &prefix,
                        const std::deque<VertexDesc> &remain) {
    //all->push_back(remain);
    //return;
    int n = remain.size();
    for (int i = 0; i < n; i++) {
      std::deque<VertexDesc> p(remain.begin() + i, remain.end());
      p.insert(p.end(), remain.begin(), remain.begin() + i);
      all->push_back(p);
    }
#ifdef ALL_PERMUTATIONS    
    int n = remain.size();
    if (n == 0) {
      all->push_back(prefix);
    } else {
      for (int i = 0; i < n; i++) {
        std::deque<VertexDesc> newPrefix(prefix);
        newPrefix.push_back(remain[i]);
        std::deque<VertexDesc> newRemain(remain.begin(), remain.begin() + i);
        newRemain.insert(newRemain.end(), remain.begin() + i + 1, remain.end());
        std::cout << prefix.size() << " + " << remain.size() << " changes to -> "
                  << newPrefix.size() << " + " << newRemain.size() << std::endl;
        enumerate(all, newPrefix, newRemain);
      }
    }
#endif    
    ROS_INFO_STREAM("made " << all->size() << " enumerations");
  }

  static bool initialize_subgraph(Graph *graph,
                                  const std::deque<VertexDesc> &factors) {
    for (const auto &v: factors) { //loop over factors
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
        ROS_DEBUG_STREAM("got homography: " << tf.second << std::endl << tf.first);
        if (tf.second) {
          if (!set_value_from_tag_projection(graph, v, tf.first)) {
            return (false); // init failed!
          }
          if (!fp->isOptimized()) {
            // add factor and values to optimizer
            fp->setIsValid(true);
            fp->addToOptimizer(graph);
          }
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
      std::vector<VertexDesc> values = graph->getConnected(fac);
      for (const auto &v: values) {
        if (!graph->getVertex(v)->isOptimized()) {
          allOptimized = false;
          break;
        }
      }
    }
    return (allOptimized);
  }

  void GraphManager::initializeSubgraphs(
    std::vector<GraphPtr> *subGraphs,
    const std::vector<std::deque<VertexDesc>> &verts) {
    profiler_.reset();
    ROS_DEBUG_STREAM("----------- initializing " << verts.size() << " subgraphs");
    subGraphs->clear();
    for (const auto &vs: verts) {  // iterate over all subgraphs
      ROS_DEBUG_STREAM("---------- subgraph of size: " << vs.size());
      std::vector<std::deque<VertexDesc>> orderings;
      enumerate(&orderings, std::deque<VertexDesc>(), vs);
      double errMin = 1e10;
      GraphPtr bestGraph;
      ROS_DEBUG_STREAM("number of orderings: " << orderings.size());
      int numOrderingsTried(0);
      for (const auto &factors: orderings) {
        GraphPtr sg(new Graph());
        Graph &subGraph = *sg;
        std::deque<VertexDesc> vset;
        // This makes a deep copy, hopefully
        subGraph.copyFrom(graph_, factors, &vset);
        subGraph.print("init subgraph");
        if (initialize_subgraph(&subGraph, vset)) {
          double err =  subGraph.optimizeFull();
          double maxErr = subGraph.getMaxError();
          numOrderingsTried++;
          ROS_DEBUG_STREAM("ordering " << numOrderingsTried << " has error: " << err << " " << maxErr);
          if (maxErr >= 0 && maxErr < errMin) {
            errMin = maxErr;
            bestGraph = sg;
            ROS_DEBUG_STREAM("subgraph " << numOrderingsTried << " has minimum error: " << errMin);
            if (maxErr < maxSubgraphError_) {
              ROS_DEBUG_STREAM("breaking early due to low error");
              break;
            }
          }
        } else {
          ROS_DEBUG_STREAM("subgraph rejected!");
        }
      }
      if (bestGraph) {
        subGraphs->push_back(bestGraph);
        ROS_DEBUG_STREAM("best subgraph init found after " << numOrderingsTried << " attempts with error: " << errMin);
      } else {
        ROS_WARN_STREAM("could not initialize subgraph!");
      }
    }
    profiler_.record("initializeSubgraphs");
  }

  void
  GraphManager::processNewFactors(const ros::Time &t,
                           const std::vector<VertexDesc> &facs) {
    ROS_DEBUG_STREAM("&&&&&&&&&&&&&&&&&&&&&&&&&&&&& got " << facs.size() << " new factors for t = " << t);
    SubGraph found;
    std::vector<std::deque<VertexDesc>> sv;
    if (facs.size() == 0) {
      ROS_DEBUG_STREAM("no new factors!");
      numNoFactors_++;
      return;
    }
    sv = findSubgraphs(t, facs, &found);
    if (sv.empty()) {
      ROS_DEBUG_STREAM("no new factors activated!");
      return;
    }
    // ROS_DEBUG_STREAM("^^^^^^^^^^ checking complete graph for error before doing anything! ^^^^^^^^^^");
    // double err = graph_.getError();
    
    std::vector<GraphPtr> subGraphs;
    
    initializeSubgraphs(&subGraphs, sv);
    subgraphError_ += optimizeSubgraphs(subGraphs);
    initializeFromSubgraphs(subGraphs);
    double err = optimize();
    ROS_INFO_STREAM("sum of subgraph err: " << subgraphError_ << ", full graph error: " << err);
    graph_.transferOptimizedValues();
    
    std::cout << profiler_ << std::endl;
 
    ROS_DEBUG_STREAM("&-&-&-&-&-&-&-& done with new factors for t = " << t);
    while (!times_.empty()) {
      auto it = times_.rbegin();
      const ros::Time key = it->first;
      ROS_DEBUG_STREAM("++++++++++ handling old factors for t = " << key);
      sv = findSubgraphs(key, it->second, &found);
      initializeSubgraphs(&subGraphs, sv);
      subgraphError_ += optimizeSubgraphs(subGraphs);
      initializeFromSubgraphs(subGraphs);
      err = optimize(); // run global optimization
      ROS_INFO_STREAM("sum of subgraph err: " << subgraphError_ << ", full graph error: " << err);
      // now remove factors that have been used
      ROS_DEBUG_STREAM("removing used factors...");
      for (auto ii = times_[key].begin(); ii != times_[key].end();) {
        if (contains(found.factors, *ii)) {
          ROS_DEBUG_STREAM("removing used factor " << graph_.info(*ii) << " for time "  << key);
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
      graph_.transferOptimizedValues();
    }
    ROS_INFO_STREAM("graph after update: " << graph_.getStats() << " no factors: " << numNoFactors_);
  }

  void
  GraphManager::exploreSubGraph(const ros::Time &t,
                         VertexDesc start,
                         SubGraph *subGraph, SubGraph *found) {
    std::deque<VertexDesc> factorsToExamine;
    factorsToExamine.push_back(start);
    while (!factorsToExamine.empty()) {
      VertexDesc exFac = factorsToExamine.front();
      factorsToExamine.pop_front();
      // examine() may append new factors to factorsToExamine
      examine(t, exFac, &factorsToExamine, found, subGraph);
    }
  }


  double GraphManager::reoptimize() {
    double err = graph_.optimizeFull(true /*force*/);
    graph_.transferOptimizedValues();
    const auto errMap = graph_.getErrorMap();
    ROS_INFO_STREAM("----------- error map: -----------");
    for (const auto &v: errMap) {
      ROS_INFO_STREAM("  " << v.first << " " << *(graph_.getVertex(v.second)));
    }
    return (err);
  }

  void GraphManager::plotDebug(const ros::Time &t, const string &tag) {
    graph_.plotDebug(t, tag);
  }
}  // end of namespace
