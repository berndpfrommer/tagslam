/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/init_pose.h"
#include "tagslam/camera_intrinsics.h"
#include "tagslam/logging.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <boost/range/irange.hpp>
#include <iostream>

//#define DEBUG_POSE

namespace tagslam {
  namespace init_pose {
    using boost::irange;
#ifdef DEBUG_POSE    
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

    static void test_reprojection(const cv::Mat &ip,
                                  const cv::Mat &wp,
                                  const cv::Mat &R,
                                  const cv::Mat &tvec) {
      cv::Mat rwp = (R * wp.t()).t();
      for (int i = 0; i < rwp.rows; i++) {
        rwp.row(i) = rwp.row(i) + tvec.t();
        for (int j = 0; j < 2; j++) {
          std::cout << rwp.row(i).at<double>(j)/rwp.row(i).at<double>(2) - ip.at<cv::Vec2d>(i,0)[j] << " ";
        }
        std::cout << std::endl;
      }
    }
#endif

#ifdef ALTERNATIVE_DECOMPOSITIONS
    static void decompose_homography_simple(const cv::Mat &H,
                                            cv::Mat *R,
                                            cv::Mat *rvec, cv::Mat *tvec) {
      cv::Mat RR(3,3,CV_64F);
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
      *tvec = cv::Mat(3,1,CV_64F);
      H.col(2).copyTo(*tvec);
      *R = RR;
      std::cout << "R from simple: " << std::endl << RR << std::endl;
      std::cout << "T from simple: " << std::endl << *tvec << std::endl; 
      *rvec = cv::Mat(3,1,CV_64F);
      cv::Rodrigues(RR, *rvec);
    }

    static bool decompose_homography(
      const cv::Mat &H, cv::Mat *R, cv::Mat *rvec, cv::Mat *tvec) {
      //
      // This follows Ma "An invitation to 3D vision
      //
      cv::Mat HTH, V, SIG, tmp;
      cv::SVD::compute(H.t() * H, SIG, V, tmp);
      std::cout << "H: " << std::endl << H << std::endl;
      std::cout << "V: " << std::endl << V << std::endl;
      std::cout << "SIG^2: " << std::endl << SIG << std::endl;
      double sig1(SIG.at<double>(0)),  sig2(SIG.at<double>(1)),
        sig3(SIG.at<double>(2));
      if (fabs(sig2 - 1.0) > 1e-7) {
        BOMB_OUT("2nd singular value must be 1.0, but is " << sig2);
      }
      if (sig1 < sig2 || sig2 < sig3) {
        BOMB_OUT("singular values not sorted!");
      }
      double c1(std::sqrt(sig1  - 1.0)), c3(std::sqrt(1.0 - sig3));
      double dsig(std::sqrt(sig1 - sig3));

      cv::Mat v1 = V.col(0);
      cv::Mat v2 = V.col(1);
      cv::Mat v3 = V.col(2);
      cv::Mat u1 = (c3 * v1 + c1 * v3) / dsig;
      cv::Mat u2 = (c3 * v1 - c1 * v3) / dsig;
      cv::Mat U1(3,3, CV_64F), U2(3,3,CV_64F),
                 W1(3,3, CV_64F), W2(3,3, CV_64F);
      v2.copyTo(U1(cv::Rect(0, 0, 1, 3)));
      u1.copyTo(U1(cv::Rect(1, 0, 1, 3)));
      v2.cross(u1).copyTo(U1(cv::Rect(2, 0, 1, 3)));

      v2.copyTo(U2(cv::Rect(0, 0, 1, 3)));
      u2.copyTo(U2(cv::Rect(1, 0, 1, 3)));
      v2.cross(u2).copyTo(U2(cv::Rect(2, 0, 1, 3)));

      cv::Mat Hv2(H * v2), Hu1(H * u1), Hu2(H * u2);
      Hv2.copyTo(W1(cv::Rect(0, 0, 1, 3)));
      Hu1.copyTo(W1(cv::Rect(1, 0, 1, 3)));
      Hv2.cross(Hu1).copyTo(W1(cv::Rect(2, 0, 1, 3)));
 
      Hv2.copyTo(W2(cv::Rect(0, 0, 1, 3)));
      Hu2.copyTo(W2(cv::Rect(1, 0, 1, 3)));
      Hv2.cross(Hu2).copyTo(W2(cv::Rect(2, 0, 1, 3)));

      cv::Mat R1 = W1 * U1.t();
      cv::Mat N1 = v2.cross(u1);
      cv::Mat T1 = (H - R1) * N1;
      std::cout << "R1: " << std::endl << R1 << std::endl;
      std::cout << "N1: " << std::endl << N1 << std::endl;
      std::cout << "T1: " << std::endl << T1 << std::endl;

      cv::Mat R2 = W2 * U2.t();
      cv::Mat N2 = v2.cross(u2);
      // must add term: R * e3 for shift to zero coordinates
      cv::Mat T2 = (H - R2) * N2;
      std::cout << "R2: " << std::endl << R2 << std::endl;
      std::cout << "N2: " << std::endl << N2 << std::endl;
      std::cout << "T2: " << std::endl << T2 << std::endl;

      // which ever solution has larger N along z component wins
      bool hasGoodR1 = R1.at<double>(2,2) < 0;
      bool hasGoodR2 = R2.at<double>(2,2) < 0;
      bool use1 = hasGoodR1 && (fabs(N1.at<double>(2))
                                > fabs(N2.at<double>(2)) || !hasGoodR2);
      *R = use1 ? R1 : R2;
      cv::Mat N    = use1 ? N1 : N2;
      cv::Mat Traw = use1 ? T1 : T2;
      cv::Mat T  = ((N.at<double>(2) < 0) ? -Traw : Traw) + R->col(2);
      *tvec = T;
      std::cout << "R: " << std::endl << *R << std::endl;
      std::cout << "T: " << std::endl << *tvec << std::endl; 
      *rvec = cv::Mat(3,1,CV_64F);
      cv::Rodrigues(*R, *rvec);
      return ((hasGoodR1 || hasGoodR2) && T.at<double>(2) > 0);
    }

#endif // end of ALTERNATIVE_DECOMPOSITIONS

    static void decompose_homography_svd(const cv::Mat &H,
                                         cv::Mat *R,
                                         cv::Mat *rvec, cv::Mat *tvec) {
      //
      // Taken from Kostas Daniliidis' MEAM 620 lecture notes
      //
      cv::Mat RR(3,3,CV_64F);
      cv::Mat H12 = cv::Mat_<double>(3,3);
      H.col(0).copyTo(H12.col(0));
      H.col(1).copyTo(H12.col(1));
      H.col(0).cross(H.col(1)).copyTo(H12.col(2));
      cv::Mat W, U, VT;
      cv::SVD::compute(H12, W, U, VT);
      cv::Mat diag = (cv::Mat_<double>(3,3) << 1, 0, 0,
                      0, 1, 0,   0, 0, cv::determinant(U*VT));
      cv::Mat RSVD = U * diag * VT;

      RR = RSVD;
      *tvec = cv::Mat(3,1,CV_64F);
      cv::Mat hl(3, 1, CV_64F);
      H.col(2).copyTo(hl);
      hl = hl / cv::norm(H.col(0));
      hl.copyTo(*tvec);
      *R = RR;
#ifdef DEBUG_POSE      
      std::cout << "R from SVD: " << std::endl << RR << std::endl;
      std::cout << "T from SVD: " << std::endl << *tvec << std::endl;
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
        BOMB_OUT("ERROR: invalid distortion model: " << distModel);
        break;
      }
      return (im);
    }
 
    static bool find_pose_by_homography(const cv::Mat &wp, const cv::Mat &ip,
                                        cv::Mat *rvec,  cv::Mat *tvec) {
      cv::Mat wc = wp.clone();
      cv::Mat wo = wc(cv::Rect(0,0,2,wc.rows)); 
#ifdef DEBUG_POSE      
      std::cout << "world points: " << cv_info(wp) << std::endl << wp << std::endl;
      std::cout << "image points: " << std::endl << ip << std::endl;
#endif
      // use opencv to get the homography
      cv::Mat HL;
      HL = cv::findHomography(wo, ip);
#ifdef DEBUG_POSE      
      std::cout << "HL from getHomography" << std::endl << HL << std::endl;
#endif
      // first normalize the homography matrix (Ma, pg 136 top) so it has form
      //     H = R + 1/d*T*N^T
      cv::Mat SL;
      cv::SVD::compute(HL, SL);
      cv::Mat H  = HL * 1.0/SL.at<double>(0, 1);
      // H has scale invariance and scale can be negative! We can ensure that
      // x_j^T * H * x_i > 0  (Ma, top of page 136) by testing
      // that the trace is positive, and flipping sign
      // if it is not.
      cv::Mat iph = cv::Mat::ones(ip.rows, 3, CV_64F);
      cv::Mat wph = cv::Mat::ones(ip.rows, 4, CV_64F);
      for (int i = 0; i < ip.rows; i++) {
        iph.at<double>(i, 0) = ip.at<cv::Vec2d>(i,0)[0];
        iph.at<double>(i, 1) = ip.at<cv::Vec2d>(i,0)[1];
      }
      cv::Mat wh = wp.clone();
      wh.col(2) = 1.0;
      cv::Mat d = iph * H * wh.t();
      double sumDiag = cv::trace(d)[0];
      cv::Mat Hpos = (sumDiag >= 0)? H : -H;
      //std::cout << "raw homography: " << std::endl << Hpos << std::endl;
      cv::Mat R;
#ifdef DEBUG_POSE
      bool foo = decompose_homography(Hpos, &R, rvec, tvec);
      std::cout << "fancy homography reprojection test: " << std::endl;
      test_reprojection(ip, wp, R, *tvec);
      decompose_homography_simple(Hpos, &R, rvec, tvec);
      std::cout << "simple homography reprojection test: " << std::endl;
      test_reprojection(ip, wp, R, *tvec);
#endif
      // use standard svd decomposition
      decompose_homography_svd(Hpos, &R, rvec, tvec);
#ifdef DEBUG_POSE
      std::cout << "svd homography reprojection test: " << std::endl;
      test_reprojection(ip, wp, R, *tvec);
#endif      
      bool rc = true;
      return (rc);
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
#ifdef DEBUG_POSE      
      std::cout << "uv points: " << std::endl << imgPoints << std::endl;
#endif      
      const cv::Mat imu = undistort_points(ip, K, distModel, D);
      cv::Mat rvec, tvec;
      bool rc =  find_pose_by_homography(wp, imu, &rvec, &tvec);
      const cv::Mat D0  = cv::Mat::zeros(0, 4, CV_64F);
      const cv::Mat eye = cv::Mat::eye(3, 3, CV_64F);
      rc = cv::solvePnP(wp, imu, eye, D0,
                        rvec,  tvec, rc, CV_ITERATIVE);

      Point3d rvecT, tvecT;
      cv2eigen(rvec, rvecT);
      cv2eigen(tvec, tvecT);
      tf.first = make_transform(rvecT, tvecT);
      tf.second = rc;
      return (tf);
    }
  } // end of namespace pnp
}
