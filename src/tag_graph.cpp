/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/tag_graph.h"
#include <boost/range/irange.hpp>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/ReferenceFrameFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

namespace tagslam {
  typedef gtsam::GenericProjectionFactor<gtsam::Pose3,
                                         gtsam::Point3,
                                         gtsam::Cal3DS2> ProjectionFactor;
  using boost::irange;
  static const gtsam::Symbol sym_T_w_o(int tagId) {
    return (gtsam::Symbol('s', tagId));
  }
  static const gtsam::Symbol sym_T_s_o(int tagId) {
    return (gtsam::Symbol('t', tagId));
  }
  static const gtsam::Symbol sym_X_o_i(int tagType, int corner) {
    return (gtsam::Symbol('o', tagType * 4 + corner));
  }
  static const gtsam::Symbol sym_X_s_i(int tagType, int corner) {
    return (gtsam::Symbol('x', tagType * 4 + corner));
  }
  static const gtsam::Symbol sym_X_w_i(int tagId, int corner) {
    return (gtsam::Symbol('w', tagId * 4 + corner));
  }

  static const gtsam::Symbol sym_T_c_t(int camId, unsigned int frame_num) {
    return (gtsam::Symbol('a' + camId, frame_num));
  }

  bool TagGraph::hasStaticObject(const std::string &name) const {
    return (staticObjects_.count(name) != 0);
  }

#if 0 
  void TagGraph::addStaticObject(const std::string &objectName,
                                 const gtsam::Pose3 &pose,
                                 const utils::PoseNoise &poseNoise) {
    staticObjects_.insert(std::map<std::string, int>::value_type(objectName,
                                                                 staticObjects_.size()));
    int objectIdx = staticObjects_.size() - 1;
    gtsam::Symbol T_w_s = sym_T_w_s(objectIdx);
    graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(T_w_s, pose, poseNoise));
    values_.insert(T_w_s, pose);
    ROS_INFO_STREAM("added static object: " << objectName);
  }
#endif

  void TagGraph::addTags(const std::string &objectName, const gtsam::Pose3 &objPose,
                         const utils::PoseNoise &objPoseNoise, const std::vector<Tag> &tags) {
    IsotropicNoisePtr smallNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-4);
    for (const auto &tag: tags) {
      std::cout << "graph tag: adding new tag: " << tag.id << std::endl;
      gtsam::Pose3   tagPose = tag.pose;
      Tag::PoseNoise tagNoise = tag.noise;
      // ----- insert transform T_s_o and pin it down with prior factor
      gtsam::Symbol T_s_o_sym = sym_T_s_o(tag.id);
      if (values_.find(T_s_o_sym) != values_.end()) {
        std::cout << "ERROR: duplicate tag id inserted: " << tag.id << std::endl;
        return;
      }
      values_.insert(T_s_o_sym, tagPose);
      std::cout << "tag has T_s_o: " << tagPose << std::endl;
      graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(T_s_o_sym, tagPose, tagNoise));
      // ------ now object-to-world transform
      // T_w_o  = T_w_s * T_s_o
      gtsam::Symbol T_w_o_sym = sym_T_w_o(tag.id);
      gtsam::Pose3  T_w_o     = objPose * tagPose;
      std::cout << "tag has T_w_o: " << T_w_o << std::endl;
      values_.insert(T_w_o_sym, T_w_o);
      graph_.push_back(gtsam::BetweenFactor<gtsam::Pose3>(T_s_o_sym, T_w_o_sym, objPose, objPoseNoise));
            
      // ------ insert the corner positions
      for (const auto i: irange(0, 4)) {
        gtsam::Symbol X_o_i      = sym_X_o_i(tag.type, i);
        gtsam::Point3 X_obj_corn = insertTagType(tag, i);
        gtsam::Symbol X_w_i      = sym_X_w_i(tag.id, i);
        values_.insert(X_w_i, T_w_o * X_obj_corn);
        graph_.push_back(gtsam::ReferenceFrameFactor<gtsam::Point3,
                         gtsam::Pose3>(X_o_i, T_w_o_sym, X_w_i, smallNoise));
      }
    }
  }

  gtsam::Point3 TagGraph::insertTagType(const Tag &tag, int corner) {
    gtsam::Symbol       X_o_i = sym_X_o_i(tag.type, corner);
    const gtsam::Point3 X = tag.getObjectCorner(corner);
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

  void TagGraph::addCamera(int cam_idx, const std::vector<double> &intr,
                           const std::string &distModel,
                           const std::vector<double> &distCoeff) {
    if (cameras_.count(cam_idx) > 0) {
      throw std::runtime_error("duplicate cam idx!" + std::to_string(cam_idx));
    }
    if (distCoeff.size() > 4) {
      throw std::runtime_error("max of 4 dist coeff is supported!");
    }
    double dc[4] = {0, 0, 0, 0};
    for (const auto i: irange(0ul, distCoeff.size())) {
      dc[i] = distCoeff[i];
    }
    boost::shared_ptr<gtsam::Cal3DS2> cam(
      new gtsam::Cal3DS2(intr[0], intr[1], 0.0, intr[2], intr[3],
                         dc[0], dc[1], dc[2], dc[3]));
    cameras_.insert(CamMap::value_type(cam_idx, GraphCam(cam, intr,
                                                         distModel, distCoeff)));
  }

  bool TagGraph::getPoints(std::vector<cv::Point2f> *img_pts,
                           std::vector<cv::Point3f> *world_pts,
                           const std::vector<Tag> &tags) const {
    img_pts->clear();
    world_pts->clear();
    // 
    for (const Tag &tag: tags) {
      for (const auto i: irange(0, 4)) {
        const gtsam::Symbol sym = sym_X_w_i(tag.id, i);
        gtsam::Values::const_iterator it = values_.find(sym);
        if (it == values_.end()) {
          continue; // tag is not in graph yet because it has not been localized!
        }
        gtsam::Point3 X_w = values_.at<gtsam::Point3>(it->key); 
        gtsam::Point2 uv  = tag.corners[i];
        world_pts->push_back(cv::Point3f(X_w.x(), X_w.y(), X_w.z()));
        img_pts->push_back(cv::Point2f(uv.x(), uv.y()));
      }
    }
    return (!world_pts->empty());
  }



  void TagGraph::observedTags(int cam_idx, const std::vector<Tag> &tags,
                              unsigned int frame_num,
                              const gtsam::Pose3 &T_w_c_guess) {
    std::cout << "-- observed tags for cam " << cam_idx << " frame " << frame_num << " tags: " << tags.size() << std::endl;
    std::cout << "initial camera pose: " << std::endl << T_w_c_guess << std::endl;
    if (cam_idx < 0 || cam_idx >= cameras_.size()) {
      std::cout << "ERROR: invalid camera index!" << std::endl;
      return;
    }
    if (tags.empty()) {
      std::cout << "WARN: no tags for cam " << cam_idx << " in frame " << frame_num << std::endl;
      return;
    }
    gtsam::Symbol T_w_c_sym = sym_T_c_t(cam_idx, frame_num);
    values_.insert(T_w_c_sym, T_w_c_guess);

    IsotropicNoisePtr pixelNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
    for (const auto &tag: tags) {
      for (const auto i: irange(0, 4)) {
        gtsam::Symbol X_w_i = sym_X_w_i(tag.id, i);
        // This tag may not be in the graph yet because its pose
        // could not be estimated due to lack of camera pose!
        if (values_.exists(X_w_i)) {
          gtsam::Point2 uv(tag.corners[i].x(), tag.corners[i].y());
          graph_.push_back(ProjectionFactor(uv, pixelNoise,
                                            T_w_c_sym, X_w_i,
                                            cameras_[cam_idx].gtsamCameraModel));
        } else {
          std::cout << "tag " << tag.id << " not observed yet!" << std::endl;
        }
      }
    }
    std::cout << frame_num << " cam " << cam_idx << " observed " << tags.size() << " tags" << std::endl;
    optimize();
    updateCameraPoses(frame_num);
  }

  static double tryOptimization(gtsam::Values *result,
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
      double err = lmo.error();
      return (err);
  }

  void
  TagGraph::updateCameraPoses(unsigned int frameNum) {
    for (const auto cam_idx: irange(0ul, cameras_.size())) {
      const gtsam::Symbol sym = sym_T_c_t(cam_idx, frameNum);
      if (optimizedValues_.find(sym) != optimizedValues_.end()) {
        cameras_[cam_idx].lastPose = optimizedValues_.at<gtsam::Pose3>(sym.key());
        cameras_[cam_idx].hasValidPose = true;
      }
    }
  }

  void
  TagGraph::getCameraPoses(std::vector<std::pair<int, gtsam::Pose3>> *poses,
                           unsigned int frame_num) const {
    for (const auto &cam: cameras_) {
      if (cam.second.hasValidPose) {
        poses->push_back(std::pair<int, gtsam::Pose3>(cam.first, cam.second.lastPose));
      }
    }
  }
  
  bool
  TagGraph::getCameraPose(int cam_idx, gtsam::Pose3 *pose) const {
    const auto it = cameras_.find(cam_idx);
    if (it != cameras_.end() && it->second.hasValidPose) {
      *pose = it->second.lastPose;
      return (true);
    }
    return (false);
  }


  void
  TagGraph::getTagPoses(std::vector<std::pair<int, gtsam::Pose3>> *poses) const {
    gtsam::Values::const_iterator it_start = optimizedValues_.lower_bound(gtsam::Symbol('s', 0));
    gtsam::Values::const_iterator it_stop  = optimizedValues_.upper_bound(gtsam::Symbol('s', 1024*1024));
    for (gtsam::Values::const_iterator it = it_start; it != it_stop; ++it) {
      gtsam::Symbol sym(it->key);
      poses->push_back(std::pair<int, gtsam::Pose3>(sym.index(),
                                                    optimizedValues_.at<gtsam::Pose3>(it->key)));
    }
  }


  void TagGraph::optimize() {
    //graph_.print();
    //values_.print();
    double err = tryOptimization(&optimizedValues_, graph_, values_, "TERMINATION", 100);
    //result.print();
  }

}  // namespace
