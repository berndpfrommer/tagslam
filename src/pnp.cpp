/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/pnp.h"
#include "tagslam/camera_intrinsics2.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <boost/range/irange.hpp>
#include <iostream>

namespace tagslam {
  namespace pnp {
    using boost::irange;
    std::string cv_type_to_str(int type) {
      std::string r;
      uchar depth = type & CV_MAT_DEPTH_MASK;
      uchar chans = 1 + (type >> CV_CN_SHIFT);
      switch ( depth ) {
      case CV_8U:  r = "8U"; break;
      case CV_8S:  r = "8S"; break;
      case CV_16U: r = "16U"; break;
      case CV_16S: r = "16S"; break;
      case CV_32S: r = "32S"; break;
      case CV_32F: r = "32F"; break;
      case CV_64F: r = "64F"; break;
      default:     r = "User"; break;
      }
      r += "C";
      r += (chans+'0');
      return r;
    }

    std::string cv_info(const cv::Mat &m) {
      std::stringstream ss;
      std::vector<cv::Mat> c(m.channels());
      cv::split(m, c);
      ss << cv_type_to_str(m.type()) << " (" << m.cols << "x" << m.rows << ")";
      for (int i = 0; i < (int)c.size(); i++) {
        cv::Point minLoc, maxLoc;
        double minVal, maxVal;
        cv::minMaxLoc(c[i], &minVal, &maxVal, &minLoc, &maxLoc);
        ss << "[" << minVal << " - " << maxVal << "]";
      }
      return (ss.str());
    }

    static void decompose_homography(const cv::Mat &HIN,
                                     cv::Mat *rvec, cv::Mat *tvec) {
      // H has scale invariance and scale can be negative! We can ensure that
      // x_j^T * H * x_i > 0  (Ma, top of page 136) by testing for the (z,z)
      // component of H, and flipping sign on H if it is negative.
      cv::Mat HL = (HIN.at<double>(2,2) >= 0)? HIN : -HIN;
      // first normalize the homography matrix so it has form
      //     H = R + 1/d*T*N^T
      cv::Mat SL;
      cv::SVD::compute(HL, SL);
      cv::Mat H  = HL * 1.0/SL.at<double>(0, 1);
      cv::Mat RR(3,3,CV_64F);
#ifdef USE_SHORTCUT      
      // now normalize and orthogonalize the first two columns,
      // which are the first two columns of the rotation matrix R
      cv::Mat r1 = H.col(0)/cv::norm(H.col(0));
      cv::Mat r2 = H.col(1) - H.col(1).dot(r1)*r1;
      r2 = r2/cv::norm(r2);
      r1.copyTo(RR.col(0));
      r2.copyTo(RR.col(1));
      // r3 = r1 x r2
      RR.at<double>(0,2) = r1.at<double>(1,0)*r2.at<double>(2,0) -
        r1.at<double>(2,0)*r2.at<double>(1,0);
      RR.at<double>(1,2) = r1.at<double>(2,0)*r2.at<double>(0,0) -
        r1.at<double>(0,0)*r2.at<double>(2,0);
      RR.at<double>(2,2) = r1.at<double>(0,0)*r2.at<double>(1,0) -
        r1.at<double>(1,0)*r2.at<double>(0,0);
      //std::cout << "decomposed homography simple: " << std::endl << RR << std::endl;
      *tvec = cv::Mat(3,1,CV_64F);
      H.col(2).copyTo(*tvec);
#else      
      cv::Mat H12 = cv::Mat_<double>(3,3);
      HL.col(0).copyTo(H12.col(0));
      HL.col(1).copyTo(H12.col(1));
      HL.col(0).cross(HL.col(1)).copyTo(H12.col(2));
      cv::Mat W, U, VT;
      cv::SVD::compute(H12, W, U, VT);
      cv::Mat diag = (cv::Mat_<double>(3,3) << 1, 0, 0,    0, 1, 0,   0, 0, cv::determinant(U*VT));
      cv::Mat RSVD = U * diag * VT;
      //std::cout << "decomposed homography SVD: " << cv::determinant(RSVD) << std::endl << RSVD << std::endl;
      //std::cout << "error vs homography SVD: " << W.at<double>(0) << " " << W.at<double>(1) << " " << W.at<double>(2) << std::endl << RSVD * RR.t() << std::endl;
      RR = RSVD;
      *tvec = cv::Mat(3,1,CV_64F);
      cv::Mat hl(3, 1, CV_64F);
      HL.col(2).copyTo(hl);
      hl = hl / cv::norm(HL.col(0));
      hl.copyTo(*tvec);
#endif      
      *rvec = cv::Mat(3,1,CV_64F);
      cv::Rodrigues(RR, *rvec);
    }

    static cv::Mat
    undistort_points(const cv::Mat &im,
                     const cv::Mat &K,  DistortionModel distModel,
                     const cv::Mat &D) {
      const cv::Mat im2chan(im.rows, 1, CV_64FC2, im.data);
      switch (distModel) {
      case RADTAN: {
        cv::Mat im_undist;
        cv::undistortPoints(im2chan, im_undist, K, D);
        return (im_undist);
        break;  }
      case EQUIDISTANT: {
        cv::Mat im_undist;
        cv::fisheye::undistortPoints(im2chan, im_undist, K, D);
        return (im_undist);
        break; }
      default:
        std::cout << "ERROR: invalid distortion model: " << distModel << std::endl;
        break;
      }
      return (im);
    }
 
    static void find_pose_by_homography(const cv::Mat &wp, const cv::Mat &ip,
                                        cv::Mat *rvec,  cv::Mat *tvec) {
      cv::Mat wo = wp.clone();
      // but the reference camera system is 1.0 away from the tag, so
      // we need to shift the coordinates all by 1.0 in the z direction
      wo.col(2) = 1.0;
      cv::Mat H = cv::findHomography(wo, ip);
      decompose_homography(H, rvec, tvec);
    }
 
    std::pair<Transform, bool>
    pose_from_4(const Eigen::Matrix<double, 4, 2> &imgPoints,
                const Eigen::Matrix<double, 4, 3> &objPoints,
                const cv::Mat &K, DistortionModel distModel,
                const cv::Mat &D) {
      std::pair<Transform, bool> tf;
      cv::Mat ip(4, 2, CV_64F);
      cv::Mat wp(4, 3, CV_64F);
      cv::eigen2cv(imgPoints, ip);
      cv::eigen2cv(objPoints, wp);
      const cv::Mat imu = undistort_points(ip, K, distModel, D);
      cv::Mat rvec, tvec;
      find_pose_by_homography(wp, imu, &rvec, &tvec);
      // XXX check for negative z and all that good stuff
      Point3d rvecT, tvecT;
      cv2eigen(rvec, rvecT);
      cv2eigen(tvec, tvecT);
      tf.first = make_transform(rvecT, tvecT);
      tf.second = true; // XXX
      return (tf);
    }
  } // end of namespace pnp
}
