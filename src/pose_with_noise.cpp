/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/pose_with_noise.h"

namespace tagslam {
  using std::string;
  static void bombout(const string &param, const string &name) {
    ROS_ERROR_STREAM("error with " << param << " for " << name);
    throw (std::runtime_error("error with " + param + " for " + name));
  }

  static std::vector<double>
  make_default_noise(double sigma_rot, double sigma_trans) {
    std::vector<double> n(36, 0.0);
    n[0]  = n[7]   = n [14] = 1.0/sigma_rot;
    n[21] = n[28]  = n[35]  = 1.0/sigma_trans;
    return (n);
  }
  // static method
  PoseWithNoise PoseWithNoise::parse(const std::string &name,
                                     const ros::NodeHandle &nh) {
    PoseWithNoise pwn;
    // ------- make transform -------------
    double posd[3], rvecd[3];
    if (!nh.getParam(name + "/position/x", posd[0]) ||
        !nh.getParam(name + "/position/y", posd[1]) ||
        !nh.getParam(name + "/position/z", posd[2])) {
      return (pwn); // no position given, fine.
    }
    if (!nh.getParam(name + "/rotvec/x", rvecd[0]) ||
        !nh.getParam(name + "/rotvec/y", rvecd[1]) ||
        !nh.getParam(name + "/rotvec/z", rvecd[2])) {
      bombout("rotvec", name);
    }
    const Eigen::Vector3d rvec = Eigen::Map<Eigen::Vector3d>(rvecd);
    const Eigen::Vector3d pos = Eigen::Map<Eigen::Vector3d>(posd);
    Transform tf = make_transform(rvec, pos);

    // --------- make noise --------------------
    std::vector<double> Rd;
    if (!nh.getParam(name + "/R", Rd)) {
      Rd = make_default_noise(1e-6 /* rot */, 1e-6 /* trans */);
    }
    if (Rd.size() != 36) {
      bombout("R size != 36", name);
    }
    const Eigen::Matrix<double, 6, 6> R =
      Eigen::Map<Eigen::Matrix<double, 6, 6> >(&Rd[0]);
    pwn = PoseWithNoise(tf, PoseNoise2::makeFromR(R), true);
    return (pwn);
  }

  std::ostream &operator<<(std::ostream &os, const PoseWithNoise &pe) {
    os << "pose: " << pe.getPose() << std::endl << "noise: " << pe.getNoise();
    //os << "pose: " << pe." noise: " << pe.getNoise().getDiagonal().transpose();
    return (os);
  }
}  // namespace
