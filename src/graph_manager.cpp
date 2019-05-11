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
#include <cmath>
#include <algorithm>
#include <fstream>
#include <queue>
#include <map>

//#define DEBUG_GRAPH

namespace tagslam {

  using boost::irange;

  void GraphManager::addBody(const Body &body) {
    // add body pose as vertex
    if (body.isStatic()) {
      const ros::Time t0(0);
      const string name = Graph::body_name(body.getName());
      if (body.getPoseWithNoise().isValid()) {
        const PoseWithNoise &pn = body.getPoseWithNoise();
        addPoseWithPrior(t0, name, pn, false);
      } else {
        graph_->addPose(t0, name, false);
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
      addPoseWithPrior(t0, name, tag.getPoseWithNoise(), false);
    } else {
      graph_->addPose(t0, name, false);
    }
  }

  bool
  GraphManager::getPose(const ros::Time &t, const string &name,
                        Transform *tf) const {
    VertexDesc v = graph_->findPose(t, name);
    if (!Graph::is_valid(v) || !graph_->isOptimized(v)) {
      return (false);
    }
    *tf = graph_->getOptimizedPose(v);
    return (true);
  }

  PoseWithNoise
  GraphManager::getCameraPoseWithNoise(const Camera2ConstPtr &cam) const {
    PoseWithNoise pwn;
    VertexDesc v = graph_->findPose(ros::Time(0), Graph::cam_name(cam->getName()));
    if (!Graph::is_valid(v) || !graph_->isOptimized(v)) {
      return (PoseWithNoise());
    }
    return (PoseWithNoise(graph_->getOptimizedPose(v), graph_->getPoseNoise(v), true));
  }
  
  VertexDesc
  GraphManager::addPrior(const ros::Time &t, const string &name,
                         const PoseWithNoise &pn) {
    std::shared_ptr<factor::AbsolutePosePrior>
      fac(new factor::AbsolutePosePrior(t, pn, name));
    VertexDesc v = fac->addToGraph(fac, graph_.get());
    fac->addToOptimizer(graph_.get());
    return (v);
  }
  
  VertexDesc
  GraphManager::addProjectionFactor(const ros::Time &t,
                                    const Tag2ConstPtr &tag,
                                    const Camera2ConstPtr &cam,
                                    const geometry_msgs::Point *imgCorners) {
    TagProjectionFactorPtr fac(
      new factor::TagProjection(t, cam, tag, imgCorners, pixelNoise_,
                                cam->getName() + "-" +
                                Graph::tag_name(tag->getId())));
    return (fac->addToGraph(fac, graph_.get()));
  }
  
  VertexDesc
  GraphManager::addPoseWithPrior(const ros::Time &t, const string &name,
                                 const PoseWithNoise &pn, bool isCamPose) {
    VertexDesc v = graph_->addPose(t, name, isCamPose);
    graph_->addToOptimizer(v, pn.getPose());
    VertexDesc pv = addPrior(t, name, pn);
    return (pv);
  }

  VertexDesc
  GraphManager::addPose(const ros::Time &t, const string &name,
                        bool isCamPose) {
    if (graph_->hasPose(t, name)) {
      ROS_ERROR_STREAM("duplicate pose added, id: " << t << " " << name);
      throw std::runtime_error("duplicate pose added!");
    }
    VertexDesc npv = graph_->addPose(t, name, isCamPose);
    return (npv);
  }

  VertexDesc
  GraphManager::addBodyPoseDelta(const ros::Time &tPrev,
                                 const ros::Time &tCurr,
                                 const BodyConstPtr &body,
                                 const PoseWithNoise &deltaPose) {
    Transform prevPose;
    string      name = Graph::body_name(body->getName());
    VertexDesc pp = graph_->findPose(tPrev, name);
    VertexDesc cp = graph_->findPose(tCurr, name);
    
    if (!Graph::is_valid(pp)) {
      ROS_DEBUG_STREAM("adding previous pose for " << name << " " << tPrev);
      pp = addPose(tPrev, name, false);
    }
    if (!Graph::is_valid(cp)) {
      ROS_DEBUG_STREAM("adding current pose for " << name << " " << tCurr);
      cp = addPose(tCurr, name, false);
    }
    RelativePosePriorFactorPtr fac(new factor::RelativePosePrior(tCurr, tPrev, deltaPose, name));
    return (fac->addToGraph(fac, graph_.get()));
  }

  VertexDesc GraphManager::addRelativePosePrior(const  RelativePosePriorFactorPtr &fac) {
    return (fac->addToGraph(fac, graph_.get()));
  }

}  // end of namespace
