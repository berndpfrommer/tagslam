/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/factor/tag_projection.h"
#include "tagslam/graph.h"
#include "tagslam/body.h"
#include "tagslam/camera.h"
#include <geometry_msgs/Point.h>
#include <sstream>

namespace tagslam {
  namespace factor {
    TagProjection::TagProjection(const ros::Time &t,
                                 const std::shared_ptr<const Camera> &cam,
                                 const std::shared_ptr<const Tag> &tag,
                                 const geometry_msgs::Point *imgCorn,
                                 double pxn,
                                 const string   &name) :
      Factor(name, t), cam_(cam), tag_(tag), pixelNoise_(pxn) {
      imgCorners_ <<
        imgCorn[0].x, imgCorn[0].y,  imgCorn[1].x, imgCorn[1].y,
        imgCorn[2].x, imgCorn[2].y,  imgCorn[3].x, imgCorn[3].y;
    }

    string TagProjection::getLabel() const {
      std::stringstream ss;
      ss << "proj:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }

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
      const VertexDesc v = g->find(this);
      checkIfValid(v, "factor not found");
      g->verifyUnoptimized(v);
      const std::vector<ValueKey> optKeys = g->getOptKeysForFactor(v, 4);
      const std::vector<FactorKey> fks = 
        g->getOptimizer()->addTagProjectionFactor(
          getImageCorners(), getTag()->getObjectCorners(),
          getCamera()->getName(), getCamera()->getIntrinsics(),
          getPixelNoise(), optKeys[0], optKeys[1], optKeys[2], optKeys[3]);
      g->markAsOptimized(v, fks);
    }
  } // namespace factor
}  // namespace tagslam
