/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <tagslam/utils.h>
#include <iomanip>
#include <opencv2/calib3d/calib3d.hpp>

namespace tagslam {
namespace utils {
  gtsam::Matrix3 rotmat(const Eigen::Vector3d &rvec) {
    Eigen::Matrix<double, 3, 1> axis{0, 0, 0};
    double angle(0);
    const double n = rvec.norm();
    if (n > 1e-8) {
      axis = rvec / n;
      angle = n;
    }
    Eigen::Matrix<double, 3, 3> m(Eigen::AngleAxis<double>(angle, axis));
    return (gtsam::Matrix3(m));
  }
    
  PoseNoise make_pose_noise(const Eigen::Vector3d &a,
                            const Eigen::Vector3d &p) {
    gtsam::Vector sn(6);
    sn << a(0),a(1),a(2),p(0),p(1),p(2);
    return (gtsam::noiseModel::Diagonal::Sigmas(sn));
  }

  PoseNoise make_pose_noise(double angle, double position) {
    gtsam::Vector sn(6);
    sn << angle,angle,angle,position,position,position;
    return (gtsam::noiseModel::Diagonal::Sigmas(sn));
  }

  // creates intrinsic matrix from 4 doubles: fx, fy, cx, cy
  static cv::Mat make_intrinsic_matrix(const double *intr) {
    return (cv::Mat_<double>(3,3) <<
            intr[0], 0,       intr[2],
            0,       intr[1], intr[3],
            0,       0,       1.0);
  }

  // hat operator, makes skew symmetric matrix from vector
  static cv::Mat hat(const cv::Mat &vi) {
    assert((vi.cols == 3 && vi.rows == 1) ||
           (vi.cols == 1 && vi.rows == 3));
    cv::Mat v = (vi.rows == 3) ? vi.t() : vi;
    return ((cv::Mat_<double>(3,3) <<
             0,                  -v.at<double>(0,2),   v.at<double>(0,1),
             v.at<double>(0,2),                  0,   -v.at<double>(0,0),
             -v.at<double>(0,1),  v.at<double>(0,0),                0.0));
  }

  // in an array of column vectors [3x1],
  // find the one with the largest z component
  static int find_max_z(const std::vector<cv::Mat> &vv) {
    double max_val = -1e10;
    int max_i = 0;
    for (int i = 0; i < (int) vv.size(); i++) {
      double vz = (vv[i].cols==3) ? vv[i].at<double>(0,2):vv[i].at<double>(2,0);
      if (vz > max_val) {
        max_i = i;
        max_val = vz;
      }
    }
    return (max_i);
  }

  // helper function to create matrix [a,b,c] from column vectors a, b, c
  static cv::Mat
  make_mat(const cv::Mat &a, const cv::Mat &b, const cv::Mat &c) {
    cv::Mat m = (cv::Mat_<double>(3,3) <<
                 a.at<double>(0,0), b.at<double>(0,0), c.at<double>(0,0),
                 a.at<double>(1,0), b.at<double>(1,0), c.at<double>(1,0),
                 a.at<double>(2,0), b.at<double>(2,0), c.at<double>(2,0));
    return (m);
  }

  // Implements homography decomposition following
  // "An Invitation to 3-d Vision" (Ma, Soatto, Kosecka, Sastry),
  // p 136 and following.
  // Of the 4 possible solutions, returns the rotation and translation of the
  // one with the largest Z component in the normal vector.

  static void decomposeHomography(const cv::Mat &HIN, const cv::Mat &K,
                                  cv::Mat *RR, cv::Mat *TT) {
    // first turn the H matrix into the normalized one by accounting
    // for the camera intrinsics
    cv::Mat KH = K.inv() * HIN;
    // H has scale invariance and scale can be negative! We can ensure that
    // x_j^T * H * x_i > 0  (Ma, top of page 136) by testing for the (z,z)
    // component of H, and flipping sign on H if it is negative.
    cv::Mat HL = (KH.at<double>(2,2) >= 0)? KH : -KH;
    // first normalize the homography matrix so it has form
    //     H = R + 1/d*T*N^T
    cv::Mat SL;
    cv::SVD::compute(HL, SL);
    cv::Mat H  = HL * 1.0/SL.at<double>(0, 1);
    // now normalize and orthogonalize the first two columns,
    // which are the first two columns of the rotation matrix R
    cv::Mat r1=H.col(0)/cv::norm(H.col(0));
    cv::Mat r2 = H.col(1) - H.col(1).dot(r1)*r1;
    r2 = r2/cv::norm(r2);
    *RR = cv::Mat(3,3,CV_64F);
    r1.copyTo(RR->col(0));
    r2.copyTo(RR->col(1));
    // r3 = r1 x r2
    RR->at<double>(0,2) = r1.at<double>(1,0)*r2.at<double>(2,0) -
      r1.at<double>(2,0)*r2.at<double>(1,0);
    RR->at<double>(1,2) = r1.at<double>(2,0)*r2.at<double>(0,0) -
      r1.at<double>(0,0)*r2.at<double>(2,0);
    RR->at<double>(2,2) = r1.at<double>(0,0)*r2.at<double>(1,0) -
      r1.at<double>(1,0)*r2.at<double>(0,0);
    *TT = cv::Mat(3,1,CV_64F);
    H.col(2).copyTo(*TT);
    *TT = TT->t();
  }

  static void print_vec(const std::string &s,
                        const std::vector<cv::Point2f> &v) {
    std::cout << s << " = [ ..." << std::endl;;
    for (int i = 0; i < v.size(); i++) {
      std::cout << v[i].x << ", " << v[i].y;
      if (i < v.size() - 1) {
        std::cout << "; ..." << std::endl;
      } else {
        std::cout << "];" << std::endl;
      }
    }
  }

  // computes a pose (rotation vector, translation) via homography
  // from world and image points. Distortion is not taken into account,
  // obviously this is just a starting guess.
  gtsam::Pose3 get_init_pose(const std::vector<gtsam::Point3> &world_points,
                             const std::vector<gtsam::Point2> &image_points,
                             const Eigen::Matrix<double, 3, 3> &K) {
    cv::Mat KCV = (cv::Mat_<double>(3,3) <<
                   K(0,0), K(0,1), K(0,2),
                   K(1,0), K(1,1), K(1,2),
                   K(2,0), K(2,1), K(2,2));
    //std::cout << "K = " << KCV << std::endl;
    std::vector<cv::Point2f> wp, ip;
    for (const auto &w: world_points) {
      wp.push_back(cv::Point2f(w.x(), w.y()));
    }
    //print_vec("wp", wp);
    for (const auto &i: image_points) {
      ip.push_back(cv::Point2f(i.x(), i.y()));
    }
    //print_vec("ip", ip);
    // use opencv to calculate the homography matrix. That works,
    // but for the life of it I couldn't get the decomposition to work.
    cv::Mat H = cv::findHomography(wp, ip);

    // now decompose the homography matrix following Ma's book.
    cv::Mat RO, TO; // optimal rotation and translation
    decomposeHomography(H, KCV, &RO, &TO);

    // The homography computes a transform from camera frame 1 to frame 2, i.e:
    //         X_2 = R * X_1 + T
    // In camera frame 1, the calibration target is assumed to be at a
    // plane z = +1 away from the camera. The calibration code however assumes
    // the points are at z = 0. To get the right transform we have to first add
    // [0,0,1]' to X_1, which is equivalent to adding R * [0,0,1]' to T.
    //
    //TO = (cv::Mat_<double>(3,1) << 0.2,-0.1,5.0)).t();
    TO = TO;
    //TO = TO - (RO*(cv::Mat_<double>(3,1) << 0,0,1.0)).t();

    // convert to rotation vector
    Eigen::Matrix<double, 3, 3> m;
    m(0,0) = RO.at<double>(0,0);
    m(0,1) = RO.at<double>(0,1);
    m(0,2) = RO.at<double>(0,2);
    m(1,0) = RO.at<double>(1,0);
    m(1,1) = RO.at<double>(1,1);
    m(1,2) = RO.at<double>(1,2);
    m(2,0) = RO.at<double>(2,0);
    m(2,1) = RO.at<double>(2,1);
    m(2,2) = RO.at<double>(2,2);

    gtsam::Rot3  R(m);
    Eigen::Vector3d v{TO.at<double>(0,0), TO.at<double>(0,1), TO.at<double>(0,2)};
    gtsam::Point3 T(v);
    gtsam::Pose3  trans(R, T);
    return (trans);
  }

  gtsam::Pose3 get_init_pose_pnp(const std::vector<cv::Point3f> &world_points,
                                 const std::vector<cv::Point2f> &image_points,
                                 const cv::Mat &K, const cv::Mat &D,
                                 bool *success) {
    cv::Mat rvec, tvec;
    bool sc = cv::solvePnP(world_points, image_points, K, D,
                           rvec, tvec);
    if (!sc) {
      std::cout << "WARNING: solvePnP failed!" << std::endl;
    }
    if (success) {
      *success = sc;
    }

    std::cout << "rvec: " << rvec << std::endl;
    std::cout << "tvec: " << tvec << std::endl;
    gtsam::Vector tvec_gtsam = (gtsam::Vector(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)).finished();
    gtsam::Pose3 pose(gtsam::Rot3::rodriguez(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)), tvec_gtsam);
    return (pose);
  }
}
}
