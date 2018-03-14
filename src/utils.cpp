/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <tagslam/utils.h>
#include <iomanip>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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
  static cv::Mat make_intrinsic_matrix(const std::vector<double> &intr) {
    return (cv::Mat_<double>(3,3) <<
            intr[0], 0,       intr[2],
            0,       intr[1], intr[3],
            0,       0,       1.0);
  }

  static void decomposeHomography(const cv::Mat &HIN,
                                  cv::Mat *RR, cv::Mat *TT) {
    // H has scale invariance and scale can be negative! We can ensure that
    // x_j^T * H * x_i > 0  (Ma, top of page 136) by testing for the (z,z)
    // component of H, and flipping sign on H if it is negative.
    cv::Mat HL = (HIN.at<double>(2,2) >= 0)? HIN : -HIN;
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

  // computes a pose (rotation vector, translation) via homography
  // from world and image points. Distortion is not taken into account
  // but the projection model (i.e fisheye) is.

  gtsam::Pose3 get_init_pose(const std::vector<gtsam::Point3> &world_points,
                             const std::vector<gtsam::Point2> &image_points,
                             const std::vector<double> &intrinsics,
                             const std::string &distModel,
                             const std::vector<double> &distcoeff) {
    cv::Mat K = make_intrinsic_matrix(intrinsics);

    cv::Mat dist;
    if (distcoeff.empty()) {
      dist = (cv::Mat_<double>(4,1) << 0, 0, 0, 0);
    } else {
      dist = cv::Mat_<double>(distcoeff.size(), 1);
      for (unsigned int i = 0; i < distcoeff.size(); i++) {
        dist.at<double>(i, 0) = distcoeff[i];
      }
    }

    std::vector<cv::Point2f> wp, // world points
      ip,  // image points
      ipu; // undistorted image points

    // world points
    //std::cout << "   world points: " << std::endl;
    for (const auto &w : world_points) {
      wp.push_back(cv::Point2f(w.x(), w.y()));
      //std::cout << w.x << " " << w.y << std::endl;
    }
    // image points
    //std::cout << "   image points: " << std::endl;
    for (const auto &i : image_points) {
      ip.push_back(cv::Point2f(i.x(), i.y()));
      //std::cout << i.x << " " << i.y << std::endl;
    }
    if (distModel == "equidistant") {
      cv::fisheye::undistortPoints(ip, ipu, K, dist);
    } else if (distModel == "radtan") {
      cv::undistortPoints(ip, ipu, K, dist);
      //std::cout << "using radtan!" << std::endl;
    } else {
      std::cout << "WARNING: unknown distortion model: " << distModel << std::endl;
      ipu = ip;
    }
    //std::cout << "   image undist: " << std::endl;
    //std::cout << ipu << std::endl;
    // Use opencv to calculate the homography matrix.
    cv::Mat H = cv::findHomography(wp, ipu);

    // now decompose the homography matrix
    cv::Mat RO, TO; // optimal rotation and translation
    decomposeHomography(H, &RO, &TO);
    gtsam::Point3 T(TO.at<double>(0),TO.at<double>(1),TO.at<double>(2));
    gtsam::Rot3 R(RO.at<double>(0,0), RO.at<double>(0,1), RO.at<double>(0,2),
                  RO.at<double>(1,0), RO.at<double>(1,1), RO.at<double>(1,2),
                  RO.at<double>(2,0), RO.at<double>(2,1), RO.at<double>(2,2));
    gtsam::Pose3 pose(R, T);
    return (pose);
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
    gtsam::Vector tvec_gtsam = (gtsam::Vector(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)).finished();
    gtsam::Pose3 pose(gtsam::Rot3::rodriguez(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)), tvec_gtsam);
    return (pose);
  }
}
}
