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

  bool get_init_pose(const std::vector<cv::Point3d> &world_points,
                     const std::vector<cv::Point2d> &image_points,
                     const cv::Mat &K,
                     const std::string &distModel,
                     const cv::Mat &D,
                     cv::Mat *rvec,
                     cv::Mat *tvec) {
    std::vector<cv::Point2f> wp, // world points
      ip,  // image points
      ipu; // undistorted image points

    // world points
    //std::cout << "   world points: " << std::endl;
    for (const auto &w : world_points) {
      wp.push_back(cv::Point2f(w.x, w.y));
      //std::cout << w.x << " " << w.y << std::endl;
    }
    // image points
    //std::cout << "   image points: " << std::endl;
    for (const auto &i : image_points) {
      ip.push_back(cv::Point2f(i.x, i.y));
      //std::cout << i.x << " " << i.y << std::endl;
    }
    if (distModel == "equidistant") {
      cv::fisheye::undistortPoints(ip, ipu, K, D);
    } else if (distModel == "radtan" || distModel == "plumb_bob") {
      cv::undistortPoints(ip, ipu, K, D);
      //std::cout << "using radtan!" << std::endl;
    } else {
      std::cout << "WARNING: unknown distortion model: " << distModel << std::endl;
      ipu = ip;
    }
    std::cout << "   image undist: " << std::endl;
    std::cout << ipu << std::endl;
    // Use opencv to calculate the homography matrix.
    cv::Mat H = cv::findHomography(wp, ipu);

    // now decompose the homography matrix
    cv::Mat RO, TO; // optimal rotation and translation
    decomposeHomography(H, rvec, tvec);
    return (true);
  }

  //
  // Will return T_c_w, i.e. world-to-camera transform
  //

  bool get_init_pose_pnp(const std::vector<cv::Point3d> &world_points,
                         const std::vector<cv::Point2d> &image_points,
                         const cv::Mat &K,
                         const std::string &distModel,
                         const cv::Mat &D,
                         cv::Mat *rvec,
                         cv::Mat *tvec) {
    //std::cout << "init pose pnp: K: " << K <<std::endl;
    //std::cout << "D: " << D << std::endl;
    bool status(false);
    if (distModel == "plumb_bob" || distModel == "radtan") {
      status = cv::solvePnP(world_points, image_points, K, D,
                            *rvec, *tvec, false);
    } else {
      cv::Mat im_undist;
      cv::fisheye::undistortPoints(image_points, im_undist, K, D, K);
      status = cv::solvePnP(world_points, im_undist, K, cv::Mat(),
                            *rvec, *tvec, false);
      if (!status && (image_points.size() == 4)) { // indicates failure
        *tvec = (cv::Mat_<double>(3, 1) << 0, 0, 1);
        *rvec = (cv::Mat_<double>(3, 1) << 3.141, 0, 0);
        status = cv::solvePnP(world_points, im_undist, K, cv::Mat(),
                              *rvec, *tvec, true, CV_P3P);
      }
    }
    return (status);
  }

  
  void project_points(const std::vector<cv::Point3d> &wp,
                      const cv::Mat &rvec,
                      const cv::Mat &tvec,
                      const cv::Mat &K,
                      const std::string &distModel,
                      const cv::Mat &D, 
                      std::vector<cv::Point2d> *ip) {
    if (wp.empty()) {
      *ip = std::vector<cv::Point2d>();
      return;
    }
    if (distModel == "equidistant") {
      cv::Affine3d::Vec3 arvec = rvec;
      cv::Affine3d::Vec3 atvec = tvec;
      cv::fisheye::projectPoints(wp, *ip, arvec, atvec, K, D);
    } else if (distModel == "radtan" || distModel == "plumb_bob") {
      cv::projectPoints(wp, rvec, tvec, K, D, *ip);
    } else {
      std::cout << "WARNING: unknown distortion model: " << distModel << " using radtan!" << std::endl;
      cv::projectPoints(wp, rvec, tvec, K, D, *ip);
    }
  }

  double reprojection_error(const std::vector<cv::Point3d> &wp,
                            const std::vector<cv::Point2d> &ip,
                            const cv::Mat &rvec,
                            const cv::Mat &tvec,
                            const cv::Mat &K,
                            const std::string &distModel,
                            const cv::Mat &D) {
    if (wp.empty()) {
      return (0.0);
    }
    std::vector<cv::Point2d> ipp;
    project_points(wp, rvec, tvec, K, distModel, D, &ipp);
    double err(0);
    for (unsigned int i = 0; i < ipp.size(); i++) {
      cv::Point diff = ipp[i] - ip[i];
      err += sqrt(diff.x * diff.x + diff.y * diff.y);
    }
    return (err / (double) ipp.size());
  }
  
  double get_pixel_range(const std::vector<gtsam::Point2> &ip) {
    double min_pix[2] = {1e30, 1e30};
    double max_pix[2] = {-1e30, -1e30};
    for (const auto &p: ip) {
      min_pix[0] = std::min(min_pix[0], p(0));
      min_pix[1] = std::min(min_pix[1], p(1));
      max_pix[0] = std::max(max_pix[0], p(0));
      max_pix[1] = std::max(max_pix[1], p(1));
    }
    return (std::min(max_pix[0]-min_pix[0], max_pix[1]-min_pix[1]));
  }
}
}
