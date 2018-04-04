/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_graph.h"
#include "tagslam/point_distance_factor.h"
#include <boost/range/irange.hpp>
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
  static const unsigned int MAX_CAM_ID = 5;
  // beware that MAX_BODY_IDX + MAX_CAM_ID < 'o'
  static const unsigned int MAX_BODY_IDX = 5;
  
  typedef gtsam::GenericProjectionFactor<gtsam::Pose3,
                                         gtsam::Point3,
                                         gtsam::Cal3DS2> ProjectionFactor;
  using boost::irange;
  // tag corners in object coordinates
  static const gtsam::Symbol sym_X_o_i(int tagType, int corner) {
    return (gtsam::Symbol('o', tagType * 4 + corner));
  }
  // transform from object (tag) coordinate space to world
  static const gtsam::Symbol sym_T_w_o(int tagId) {
    return (gtsam::Symbol('s', tagId));
  }
  // transform from object (tag) coordinate space to static object
  static const gtsam::Symbol sym_T_b_o(int tagId) {
    return (gtsam::Symbol('t', tagId));
  }
  // tag corners in body coordinates
  static const gtsam::Symbol sym_X_b_i(int tagId, int corner) {
    return (gtsam::Symbol('x', tagId * 4 + corner));
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
    if (bodyIdx >= MAX_BODY_IDX) {
      throw std::runtime_error("body idx exceeds MAX_BODY_IDX: " + std::to_string(bodyIdx));
    }
    return (gtsam::Symbol('a' + MAX_CAM_ID + bodyIdx, frame_num));
  }

  static unsigned int X_w_i_sym_to_index(gtsam::Key k, int *tagId, int *corner) {
    gtsam::Symbol sym(k);
    int idx = sym.index();
    unsigned int frame_num = idx / (4 * MAX_TAG_ID);
    *tagId  = (idx - frame_num * 4 * MAX_TAG_ID) / 4;
    *corner = (idx - frame_num * 4 * MAX_TAG_ID) % 4;
    return (frame_num);
  }

  void TagGraph::addTags(const RigidBodyPtr &rb, const TagVec &tags) {
    IsotropicNoisePtr smallNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-4);
    for (const auto &tag: tags) {
      std::cout << "GRAPH: adding tag: " << tag->id << std::endl;
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
      if (!rb->isStatic) {
        // ------ insert the corner positions
        for (const auto i: irange(0, 4)) {
          gtsam::Symbol X_o_i      = sym_X_o_i(tag->type, i);
          gtsam::Point3 X_obj_corn = insertTagType(tag, i);
          gtsam::Symbol X_b_i      = sym_X_b_i(tag->id, i);
          values_.insert(X_b_i, tagPose * X_obj_corn);
          graph_.push_back(gtsam::ReferenceFrameFactor<gtsam::Point3,
                           gtsam::Pose3>(X_o_i, T_b_o_sym, X_b_i, smallNoise));
        }
      } else {
        // ------ object-to-world transform
        // T_w_o  = T_w_b * T_b_o
        gtsam::Symbol T_w_o_sym = sym_T_w_o(tag->id);
        gtsam::Pose3  T_w_o     = rb->poseEstimate * tagPose;
        std::cout << "GRAPH: tag id: " << tag->id << " has tf: " << T_w_o << std::endl;
        values_.insert(T_w_o_sym, T_w_o);
        graph_.push_back(gtsam::BetweenFactor<gtsam::Pose3>(
                           T_b_o_sym, T_w_o_sym, rb->poseEstimate,
                           rb->poseEstimate.getNoise()));
            
        // ------ insert the corner positions
        for (const auto i: irange(0, 4)) {
          gtsam::Symbol X_o_i      = sym_X_o_i(tag->type, i);
          gtsam::Point3 X_obj_corn = insertTagType(tag, i);
          gtsam::Symbol X_w_i      = sym_X_w_i(tag->id, i, 0 /*frame*/);
          values_.insert(X_w_i, T_w_o * X_obj_corn);
          graph_.push_back(gtsam::ReferenceFrameFactor<gtsam::Point3,
                           gtsam::Pose3>(X_o_i, T_w_o_sym, X_w_i, smallNoise));
        }
      }
    }
  }

  void TagGraph::addDistanceMeasurements(const DistanceMeasurementVec &dmv) {
    for (const auto &dm: dmv) {
      const gtsam::Symbol k1 = sym_X_w_i(dm->tag1, dm->corner1, 0);
      const gtsam::Symbol k2 = sym_X_w_i(dm->tag2, dm->corner2, 0);
      if (values_.exists(k1) && values_.exists(k2)) {
        IsotropicNoisePtr noise = gtsam::noiseModel::Isotropic::Sigma(1, dm->noise);
        graph_.push_back(PointDistanceFactor(noise, k1, k2, dm->distance));
      }
    }
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
    gtsam::Symbol T_w_c_sym = sym_T_c_t(cam->index, frame_num);
    if (values_.find(T_w_c_sym) != values_.end()) {
      pe = PoseEstimate(values_.at<gtsam::Pose3>(T_w_c_sym), 0.0, 0);
    }
    return (pe);
  }

  void TagGraph::observedTags(const CameraPtr &cam, const RigidBodyPtr &rb,
                              const TagVec &tags,
                              unsigned int frame_num) {
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
    gtsam::Symbol T_w_c_sym = sym_T_c_t(cam->index, frame_num);
    if (!values_.exists(T_w_c_sym)) {
      values_.insert(T_w_c_sym, cam->poseEstimate.getPose());
    }

    IsotropicNoisePtr pixelNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
    IsotropicNoisePtr smallNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-4);
    if (!rb->isStatic) {
      values_.insert(sym_T_w_b(rb->index, frame_num), rb->poseEstimate.getPose());
    }
    for (const auto &tag: tags) {
      if (!tag->poseEstimate.isValid()) {
        std::cout << "TagGraph WARN: tag " << tag->id << " has invalid pose!" << std::endl;
        continue;
      }
      if (!rb->isStatic) {
        const gtsam::Pose3 &T_w_b = rb->poseEstimate.getPose();
        gtsam::Symbol T_w_b_sym = sym_T_w_b(rb->index, frame_num);
        for (const auto i: irange(0, 4)) {
          gtsam::Symbol X_w_i_sym = sym_X_w_i(tag->id, i, frame_num);
          gtsam::Symbol X_b_i_sym = sym_X_b_i(tag->id, i);
          gtsam::Point3 X_b_i = values_.at<gtsam::Point3>(X_b_i_sym);
          values_.insert(X_w_i_sym, T_w_b * X_b_i);
          graph_.push_back(gtsam::ReferenceFrameFactor<gtsam::Point3,
                           gtsam::Pose3>(X_b_i_sym, T_w_b_sym, X_w_i_sym,
                                         smallNoise));
        }
      }
      // add projection factor
      const auto &uv = tag->getImageCorners();
      for (const auto i: irange(0, 4)) {
        gtsam::Symbol X_w_i = sym_X_w_i(tag->id, i, !rb->isStatic ? frame_num : 0);
        numProjectionFactors_++;
        graph_.push_back(ProjectionFactor(uv[i], pixelNoise,
                                          T_w_c_sym, X_w_i,
                                          cam->gtsamCameraModel));
      }
    }
  }

  bool TagGraph::getBodyPose(const RigidBodyConstPtr &rb, gtsam::Pose3 *pose,
                             unsigned int frame) const {
    const auto T_w_b_sym = sym_T_w_b(rb->index, frame);
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
      lmp.setAbsoluteErrorTol(1e-7);
      lmp.setRelativeErrorTol(0);
      gtsam::LevenbergMarquardtOptimizer lmo(graph, values, lmp);
      *result = lmo.optimize();
      double ni = numProjectionFactors_ > 0 ? 1.0/numProjectionFactors_ : 1.0;
      optimizerError_ = lmo.error() * ni;
      optimizerIterations_ = lmo.iterations();
      return (optimizerError_);
  }


  gtsam::Pose3
  TagGraph::getTagWorldPose(int tagId) const {
    return (values_.at<gtsam::Pose3>(sym_T_w_o(tagId)));
  }
  
  bool
  TagGraph::getTagRelPose(const RigidBodyPtr &rb, int tagId,
                          gtsam::Pose3 *pose) const {
    if (rb->isStatic) {
      gtsam::Symbol T_w_o_sym = sym_T_w_o(tagId);
      if (rb->poseEstimate.isValid() &&
          values_.find(T_w_o_sym) != values_.end()) {
        const gtsam::Pose3 T_w_o = values_.at<gtsam::Pose3>(T_w_o_sym);
        // we need T_b_o = T_b_w * T_w_o
        *pose = rb->poseEstimate.inverse() * T_w_o;
        return (true);
      }
    } else {
      const auto T_b_o_sym = sym_T_b_o(tagId);
      if (values_.find(T_b_o_sym) != values_.end()) {
        *pose = values_.at<gtsam::Pose3>(T_b_o_sym);
        return (true);
      }
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


  void TagGraph::optimize() {
    //graph_.print();
    //values_.print();
    //double err = tryOptimization(&optimizedValues_, graph_, values_, "TERMINATION", 100);
    double err = tryOptimization(&optimizedValues_, graph_, values_, "SILENT", 100);
    values_ = optimizedValues_;
    //result.print();
  }

}  // namespace
