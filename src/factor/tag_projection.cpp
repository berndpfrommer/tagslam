/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/factor/tag_projection.h"
#include "tagslam/graph.h"
#include "tagslam/body.h"
#include "tagslam/camera2.h"
#include <geometry_msgs/Point.h>
#include <string>
#include <sstream>

namespace tagslam {
  namespace factor {
    VertexDesc TagProjection::addToGraph(const VertexPtr &vp, Graph *g) const {
      const BodyConstPtr body = getTag()->getBody(); // short hand
      const BodyConstPtr rig = getCamera()->getRig(); // short hand
      // connect: tag_body_pose, tag_pose, cam_pose, rig_pose
      const VertexDesc vtp = g->findTagPose(getTag()->getId());
      checkIfValid(vtp, "no tag pose found");
      const VertexDesc vbp = g->findBodyPose(
        body->isStatic() ? ros::Time(0) : getTime(), body->getName());
      checkIfValid(vbp, "no body pose found");
      const VertexDesc vrp = g->findBodyPose(
        rig->isStatic() ? ros::Time(0) : getTime(), rig->getName());
      checkIfValid(vbp, "no rig pose found");
      const VertexDesc vcp =
        g->findCameraPose(getTime(), getCamera()->getName());
      checkIfValid(vcp, "no camera pose found");
      const VertexDesc v = g->insertFactor(vp);
      g->addEdge(v, vcp, 0); // T_r_c
      g->addEdge(v, vrp, 1); // T_w_r
      g->addEdge(v, vbp, 2); // T_w_b
      g->addEdge(v, vtp, 3); // T_b_o
      return (v);
    }

    void TagProjection::addToOptimizer(Graph *g) const {
      g->addToOptimizer(this);
    }
    
    TagProjection::TagProjection(const ros::Time &t,
                                 const std::shared_ptr<const Camera2> &cam,
                                 const std::shared_ptr<const Tag2> &tag,
                                 const geometry_msgs::Point *imgCorn,
                                 double pxn,
                                 const std::string   &name) :
      Factor(name, t), cam_(cam), tag_(tag), pixelNoise_(pxn) {
      imgCorners_ <<
        imgCorn[0].x, imgCorn[0].y,
        imgCorn[1].x, imgCorn[1].y,
        imgCorn[2].x, imgCorn[2].y,
        imgCorn[3].x, imgCorn[3].y;
    }
    std::string TagProjection::getLabel() const {
      std::stringstream ss;
      ss << "proj:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }
  } // namespace factor
}  // namespace tagslam
