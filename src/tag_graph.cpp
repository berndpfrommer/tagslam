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
  typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> ProjectionFactor;
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
  void TagGraph::addStaticObject(const std::string &objectName, const gtsam::Pose3 &pose,
                                 const utils::PoseNoise &poseNoise) {
    staticObjects_.insert(std::map<std::string, int>::value_type(objectName, staticObjects_.size()));
    int objectIdx = staticObjects_.size() - 1;
    gtsam::Symbol T_w_s = sym_T_w_s(objectIdx);
    graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(T_w_s, pose, poseNoise));
    values_.insert(T_w_s, pose);
    ROS_INFO_STREAM("added static object: " << objectName);
  }
#endif
  
  void TagGraph::addTags(const std::string &objectName, const gtsam::Pose3 &objPose,
                         const utils::PoseNoise &objPoseNoise, const std::vector<Tag> &tags) {
#if 0    
    if (hasStaticObject(objectName)) {
      std::cout << "ERROR: duplicate static object: " << objectName << std::endl;
      return;
    }
    addStaticObject(objectName, objPose, objPoseNoise);
#endif    
#if 1
    IsotropicNoisePtr smallNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-4);
    for (const auto &tag: tags) {
      // ----- insert transform T_s_o and pin it down with prior factor
      gtsam::Symbol T_s_o_sym = sym_T_s_o(tag.id);
      if (values_.find(T_s_o_sym) != values_.end()) {
        std::cout << "ERROR: duplicate tag id inserted: " << tag.id << std::endl;
        return;
      }
      values_.insert(T_s_o_sym, tag.pose);
      graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(T_s_o_sym, tag.pose, tag.noise));
      // ------ now object-to-world transform
      // T_w_o  = T_w_s * T_s_o
      gtsam::Symbol T_w_o_sym = sym_T_w_o(tag.id);
      gtsam::Pose3  T_w_o     = objPose * tag.pose;
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
#else
    const gtsam::Pose3 T_w_s = objPose;
    for (const auto &tag: tags) {
      const gtsam::Pose3  T_s_o = tag.pose;
      // ------ insert the corner positions
      for (const auto i: irange(0, 4)) {
        gtsam::Symbol X_o_i_sym  = sym_X_o_i(tag.type, i);
        gtsam::Symbol X_s_i_sym  = sym_X_s_i(tag.id, i);
        gtsam::Symbol X_w_i_sym  = sym_X_w_i(tag.id, i);

        gtsam::Point3 X_obj_corn = insertTagType(tag, i); // inserts X_o_i_sym!
        values_.insert(X_s_i_sym,  T_s_o * X_obj_corn);
        values_.insert(X_w_i_sym,  T_w_s * T_s_o * X_obj_corn);
        graph_.push_back(gtsam::BetweenFactor<gtsam::Pose3>(X_o_i_sym, X_s_i_sym, T_s_o, tag.noise));
        graph_.push_back(gtsam::BetweenFactor<gtsam::Pose3>(X_s_i_sym, X_w_i_sym, T_w_s, objPoseNoise));
      }
    }
#endif    
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

  void TagGraph::addCamera(int cam_idx, double fx, double fy,
                           double cx, double cy,
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
      new gtsam::Cal3DS2(fx, fy, 0.0, cx, cy, 
                         dc[0], dc[1], dc[2], dc[3]));
    cameras_.insert(CamMap::value_type(cam_idx, cam));
  }

  void TagGraph::getPoints(std::vector<cv::Point2f> *img_pts,
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
          throw std::runtime_error("cannot find symbol for tag id " + std::to_string(tag.id));
        }
        gtsam::Point3 X_w = values_.at<gtsam::Point3>(it->key); 
        gtsam::Point2 uv  = tag.corners[i];
        world_pts->push_back(cv::Point3f(X_w.x(), X_w.y(), X_w.z()));
        img_pts->push_back(cv::Point2f(uv.x(), uv.y()));
      }
    }
  }



  void TagGraph::observedTags(int cam_idx, const std::vector<Tag> &tags,
                              unsigned int frame_num, const cv::Mat &K,
                              const cv::Mat &D) {
    if (cam_idx < 0 || cam_idx >= cameras_.size()) {
      std::cout << "ERROR: invalid camera index!" << std::endl;
      return;
    }
    if (tags.empty()) {
      std::cout << "WARN: no tags for cam " << cam_idx << " in frame " << frame_num << std::endl;
      return;
    }
    gtsam::Symbol T_w_c_sym = sym_T_c_t(cam_idx, frame_num);
    std::vector<cv::Point2f> img_pts;
    std::vector<cv::Point3f> world_pts;
    getPoints(&img_pts, &world_pts, tags);
    std::cout << "---------- points used: " << std::endl;
    for (const auto i: irange(0ul, img_pts.size())) {
      std::cout << i << " " << img_pts[i] << " " << world_pts[i] << std::endl;
    }
    bool success{false};
    gtsam::Pose3 T_w_c_guess = utils::get_init_pose_pnp(world_pts, img_pts, K, D, &success);
    std::cout << " ======= pnp pose: " << std::endl << T_w_c_guess << std::endl;
    if (!success) {
      std::cout << "frame " << frame_num << " cam " << cam_idx
                << ": could not find start pose guess!" << std::endl;
      return;
    }
    values_.insert(T_w_c_sym, T_w_c_guess);

    IsotropicNoisePtr pixelNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
    for (const auto &tag: tags) {
      for (const auto i: irange(0, 4)) {
        gtsam::Point2 uv(tag.corners[i].x(), tag.corners[i].y());
        gtsam::Symbol X_w_i = sym_X_w_i(tag.id, i);
        graph_.push_back(ProjectionFactor(uv, pixelNoise,
                                          T_w_c_sym, X_w_i, cameras_[cam_idx]));
      }
    }
    std::cout << frame_num << " cam " << cam_idx << " observed " << tags.size() << " tags" << std::endl;
    optimize();
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
  TagGraph::getCameraPoses(std::vector<std::pair<int, gtsam::Pose3>> *poses,
                           unsigned int frame_num) const {
    for (const auto cam_idx: irange(0ul, cameras_.size())) {
      const gtsam::Symbol sym = sym_T_c_t(cam_idx, frame_num);
      if (optimizedValues_.find(sym) != optimizedValues_.end()) {
        poses->push_back(std::pair<int, gtsam::Pose3>(cam_idx,
                                                      optimizedValues_.at<gtsam::Pose3>(sym.key())));
      }
    }
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
