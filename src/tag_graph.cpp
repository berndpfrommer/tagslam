/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_graph.h"
#include "tagslam/point_distance_factor.h"
#include "tagslam/cal3ds2u.h"
#include <boost/range/irange.hpp>
#include <gtsam/slam/expressions.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/ReferenceFrameFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

namespace tagslam {
  // you can probably bump MAX_TAG_ID without too much trouble,
  // but watch out for integer overflow on the 'w' symbol
  // when using a large number of frames.
  // TODO: safeguard against overflow!
  static const unsigned int MAX_TAG_ID = 255;
  static const unsigned int MAX_CAM_ID = 8;
  static const unsigned int MAX_BODY_ID = 'Z' - 'A' - 1;

  typedef gtsam::GenericProjectionFactor<gtsam::Pose3,
                                         gtsam::Point3,
                                         gtsam::Cal3DS2> ProjectionFactor;
  using boost::irange;
  // tag corners in object coordinates
  static const gtsam::Symbol sym_X_o_i(int tagType, int corner) {
    return (gtsam::Symbol('r', tagType * 4 + corner));
  }
  // transform from object (tag) coordinate space to static object
  static const gtsam::Symbol sym_T_b_o(int tagId) {
    return (gtsam::Symbol('t', tagId));
  }
  // tag corners in world coordinates
  static const gtsam::Symbol sym_X_w_i(int tagId, int corner,
                                       unsigned int frame) {
    if (tagId > MAX_TAG_ID) {
      throw std::runtime_error("tag id exceeds MAX_TAG_ID: " + std::to_string(tagId));
    }
    return (gtsam::Symbol('w', frame * MAX_TAG_ID * 4 + tagId * 4 + corner));
  }
  // T_w_c(t) camera-to-world transform for given frame  
  static const gtsam::Symbol sym_T_c_t(int camId, unsigned int frame_num) {
    if (camId >= MAX_CAM_ID) {
      throw std::runtime_error("cam id exceeds MAX_CAM_ID: " + std::to_string(camId));
    }
    return (gtsam::Symbol('a' + camId, frame_num));
  }
  // T_w_b(t) dynamic_body-to-world transform for given frame  
  static const gtsam::Symbol sym_T_w_b(int bodyIdx, unsigned int frame_num) {
    if (bodyIdx >= MAX_BODY_ID) {
      throw std::runtime_error("body idx exceeds MAX_BODY_ID: " + std::to_string(bodyIdx));
    }
    return (gtsam::Symbol('A' + bodyIdx, frame_num));
  }

  static unsigned int X_w_i_sym_to_index(gtsam::Key k, int *tagId, int *corner) {
    gtsam::Symbol sym(k);
    int idx = sym.index();
    unsigned int frame_num = idx / (4 * MAX_TAG_ID);
    *tagId  = (idx - frame_num * 4 * MAX_TAG_ID) / 4;
    *corner = (idx - frame_num * 4 * MAX_TAG_ID) % 4;
    return (frame_num);
  }

  int TagGraph::getMaxNumBodies() const {
    return (MAX_BODY_ID);
  }
  void TagGraph::addTags(const RigidBodyPtr &rb, const TagVec &tags) {
    IsotropicNoisePtr smallNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-4);
    for (const auto &tag: tags) {
      gtsam::Pose3   tagPose  = tag->poseEstimate;
      PoseNoise tagNoise = tag->poseEstimate.getNoise();
      // ----- insert transform T_b_o and pin it down with prior factor if known
      gtsam::Symbol T_b_o_sym = sym_T_b_o(tag->id);
      if (values_.find(T_b_o_sym) != values_.end()) {
        std::cout << "TagGraph ERROR: duplicate tag id inserted: " << tag->id << std::endl;
        return;
      }
      values_.insert(T_b_o_sym, tagPose);
      if (tag->hasKnownPose) {
        graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(T_b_o_sym, tagPose, tagNoise));
      }
      // just make sure the corners are there!
      for (const auto i: irange(0, 4)) {
        insertTagType(tag, i);
      }
    }
  }

  double distance(const gtsam::Point3 &p1, const gtsam::Point3 p2, gtsam::OptionalJacobian<1, 3> H1 = boost::none,
                  gtsam::OptionalJacobian<1, 3> H2 = boost::none) {
    const gtsam::Point3 d = p1-p2;
    double r = sqrt(d.x() * d.x() + d.y() * d.y() + d.z() * d.z());
    if (H1) *H1 << d.x() / r, d.y() / r, d.z() / r;
    if (H2) *H2 << -d.x() / r, -d.y() / r, -d.z() / r;
    return r;
  }
  void
  TagGraph::addDistanceMeasurement(const RigidBodyPtr &rb1,
                                   const RigidBodyPtr &rb2,
                                   const DistanceMeasurement &dm) {
    if (!rb1->isStatic || !rb2->isStatic) {
      std::cout << "TagGraph ERROR: non-rigid body has tag measurement!" << std::endl;
      return;
    }
    gtsam::Expression<gtsam::Pose3>  T_w_b_1(sym_T_w_b(rb1->index, 0));
    gtsam::Expression<gtsam::Pose3>  T_b_o_1(sym_T_b_o(dm.tag1));
    gtsam::Expression<gtsam::Point3> X_o_1(sym_X_o_i(dm.tag1, dm.corner1));
    gtsam::Expression<gtsam::Point3> X_w_1 = gtsam::transform_from(T_w_b_1, gtsam::transform_from(T_b_o_1, X_o_1));
    
    gtsam::Expression<gtsam::Pose3>  T_w_b_2(sym_T_w_b(rb2->index, 0));
    gtsam::Expression<gtsam::Pose3>  T_b_o_2(sym_T_b_o(dm.tag2));
    gtsam::Expression<gtsam::Point3> X_o_2(sym_X_o_i(dm.tag2, dm.corner2));
    gtsam::Expression<gtsam::Point3> X_w_2 = gtsam::transform_from(T_w_b_2, gtsam::transform_from(T_b_o_2, X_o_2));

    gtsam::Expression<double> dist = gtsam::Expression<double>(&distance, X_w_1, X_w_2);
    graph_.addExpressionFactor(dist, dm.distance, gtsam::noiseModel::Isotropic::Sigma(1, dm.noise));
  }

  gtsam::Point3 TagGraph::insertTagType(const TagConstPtr &tag, int corner) {
    gtsam::Symbol       X_o_i = sym_X_o_i(tag->type, corner);
    const gtsam::Point3 X = tag->getObjectCorner(corner);
    if (values_.find(X_o_i) == values_.end()) {
      // This is the tag corner position in object coordinates.
      // If all tags are the same size(type), there are only 4 corners
      // in the whole graph!
      IsotropicNoisePtr objNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-4);
      graph_.push_back(gtsam::PriorFactor<gtsam::Point3>(X_o_i, X, objNoise));
      values_.insert(X_o_i, X);
    }
    return (X);
  }

  PoseEstimate
  TagGraph::getCameraPose(const CameraPtr &cam,
                          unsigned int frame_num) const {
    PoseEstimate pe;
    gtsam::Symbol T_w_c_sym = sym_T_c_t(cam->index, cam->isStatic ? 0 : frame_num);
    if (values_.find(T_w_c_sym) != values_.end()) {
      pe = PoseEstimate(values_.at<gtsam::Pose3>(T_w_c_sym), 0.0, 0);
    }
    return (pe);
  }

  void TagGraph::observedTags(const CameraPtr &cam, const RigidBodyPtr &rb,
                              const TagVec &tags,
                              unsigned int frame_num) {
    //std::cout << "---------- points for cam " << cam->name << " body: " << rb->name << std::endl;
    if (tags.empty()) {
      std::cout << "TagGraph WARN: no tags for " << cam->name << " in frame "
                << frame_num << std::endl;
      return;
    }
    if (!cam->poseEstimate.isValid()) {
      std::cout << "TagGraph WARN: no pose estimate for cam " << cam->name
                <<  " in frame " << frame_num << std::endl;
      return;
    }
    if (!rb->poseEstimate.isValid()) {
      return;
    }
    // new camera location
    gtsam::Symbol T_w_c_sym = sym_T_c_t(cam->index, cam->isStatic ? 0 : frame_num);
    if (!values_.exists(T_w_c_sym)) {
      values_.insert(T_w_c_sym, cam->poseEstimate.getPose());
    }

    IsotropicNoisePtr pixelNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
    IsotropicNoisePtr smallNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-4);

    gtsam::Expression<Cal3DS2U> cK(*cam->gtsamCameraModel);

    gtsam::Symbol T_w_b_sym = sym_T_w_b(rb->index, rb->isStatic ? 0 : frame_num);
    if (!values_.exists(T_w_b_sym)) {
      const auto &pe = rb->poseEstimate;
      values_.insert(T_w_b_sym, pe.getPose());
      if (rb->isStatic) {
        graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(T_w_b_sym,
                                                          pe.getPose(), pe.getNoise()));
      }
    }
    for (const auto &tag: tags) {
      if (!tag->poseEstimate.isValid()) {
        std::cout << "TagGraph WARN: tag " << tag->id << " has invalid pose!" << std::endl;
        continue;
      }
      const auto &measured = tag->getImageCorners();
      gtsam::Expression<gtsam::Pose3>  T_b_o(sym_T_b_o(tag->id));
      gtsam::Expression<gtsam::Pose3>  T_w_b(T_w_b_sym);
      gtsam::Expression<gtsam::Pose3>  T_w_c(T_w_c_sym);
      for (const auto i: irange(0, 4)) {
        gtsam::Expression<gtsam::Point3> X_o(sym_X_o_i(tag->type, i));
        // transform_from does X_A = T_AB * X_B
        gtsam::Expression<gtsam::Point3> X_w = gtsam::transform_from(T_w_b, gtsam::transform_from(T_b_o, X_o));
        gtsam::Expression<gtsam::Point2> xp  = gtsam::project(gtsam::transform_to(T_w_c, X_w));
        gtsam::Expression<gtsam::Point2> predict(cK, &Cal3DS2U::uncalibrate, xp);
        graph_.addExpressionFactor(predict, measured[i], pixelNoise);
      }
    }
  }

  bool TagGraph::getBodyPose(const RigidBodyConstPtr &rb, gtsam::Pose3 *pose,
                             unsigned int frame) const {
    const auto T_w_b_sym = sym_T_w_b(rb->index, rb->isStatic? 0 : frame);
    if (values_.find(T_w_b_sym) != values_.end()) {
      *pose = values_.at<gtsam::Pose3>(T_w_b_sym);
      return (true);
    }
    *pose = gtsam::Pose3();
    return (false);;
  }

  double TagGraph::tryOptimization(gtsam::Values *result,
                                   const gtsam::NonlinearFactorGraph &graph,
                                   const gtsam::Values &values,
                                   const std::string &verbosity, int maxIter) {
      gtsam::LevenbergMarquardtParams lmp;
      lmp.setVerbosity(verbosity);
      lmp.setMaxIterations(maxIter);
      lmp.setAbsoluteErrorTol(1e-10);
      lmp.setRelativeErrorTol(0);
      gtsam::LevenbergMarquardtOptimizer lmo(graph, values, lmp);
      *result = lmo.optimize();
      double ni = numProjectionFactors_ > 0 ? 1.0/numProjectionFactors_ : 1.0;
      optimizerError_ = lmo.error() * ni;
      optimizerIterations_ = lmo.iterations();
      return (optimizerError_);
  }


  PoseEstimate TagGraph::getTagWorldPose(const RigidBodyConstPtr &rb,
                                         int tagId, unsigned int frame_num) const {
    PoseEstimate pe;   // defaults to invalid
    const auto T_b_o_sym = sym_T_b_o(tagId);
    if (values_.find(T_b_o_sym) != values_.end()) {
      const auto T_w_b_sym = sym_T_w_b(rb->index, rb->isStatic ? 0:frame_num);
      if (values_.find(T_w_b_sym) != values_.end()) {
        gtsam::Pose3 T_w_o = values_.at<gtsam::Pose3>(T_w_b_sym) *
          values_.at<gtsam::Pose3>(T_b_o_sym);
        pe = PoseEstimate(T_w_o, 0.0, 0);
      }
    }
    return (pe);
  }
  
  bool
  TagGraph::getTagRelPose(const RigidBodyPtr &rb, int tagId,
                          gtsam::Pose3 *pose) const {
    const auto T_b_o_sym = sym_T_b_o(tagId);
    if (values_.find(T_b_o_sym) != values_.end()) {
      *pose = values_.at<gtsam::Pose3>(T_b_o_sym);
      return (true);
    }
    return (false);
  }

  void
  TagGraph::printDistances() const {
    const auto symMin = sym_X_w_i(0, 0, 0);
    const auto symMax = sym_X_w_i(MAX_TAG_ID, 3, 0);
    gtsam::Values::const_iterator it_start = values_.lower_bound(symMin);
    gtsam::Values::const_iterator it_stop  = values_.upper_bound(symMax);
    for (gtsam::Values::const_iterator it1 = it_start; it1 != it_stop; ++it1) {
      gtsam::Point3 p1 = values_.at<gtsam::Point3>(it1->key);
      int tagid, corn;
      X_w_i_sym_to_index(it1->key, &tagid, &corn);
      printf("tag %3d corner %d:", tagid, corn);
      for (gtsam::Values::const_iterator it2 = it_start; it2 != it_stop; ++it2) {
        gtsam::Point3 p2 = values_.at<gtsam::Point3>(it2->key);
        printf(" %7.4f", p1.distance(p2));
      }
      printf("\n");
    }
  }

/*  
  void
  TagGraph::getTagWorldPoses(std::vector<std::pair<int, gtsam::Pose3>> *poses) const {
    gtsam::Values::const_iterator it_start = values_.lower_bound(gtsam::Symbol('s', 0));
    gtsam::Values::const_iterator it_stop  = values_.upper_bound(gtsam::Symbol('s', 1024*1024));
    for (gtsam::Values::const_iterator it = it_start; it != it_stop; ++it) {
      gtsam::Symbol sym(it->key);
      poses->push_back(std::pair<int, gtsam::Pose3>(sym.index(),
                                                    values_.at<gtsam::Pose3>(it->key)));
    }
  }
*/

  void TagGraph::optimize() {
    //graph_.print();
    //values_.print();
    //double err = tryOptimization(&optimizedValues_, graph_, values_, "TERMINATION", 100);
    double err = tryOptimization(&optimizedValues_, graph_, values_, "TERMINATION", 100);
    values_ = optimizedValues_;
    //optimizedValues_.print();
  }

}  // namespace
