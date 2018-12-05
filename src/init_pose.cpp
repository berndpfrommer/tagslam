/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/init_pose.h"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <boost/range/irange.hpp>

namespace tagslam {
  namespace init_pose {
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
      // now normalize and orthogonalize the first two columns,
      // which are the first two columns of the rotation matrix R
      cv::Mat r1=H.col(0)/cv::norm(H.col(0));
      cv::Mat r2 = H.col(1) - H.col(1).dot(r1)*r1;
      r2 = r2/cv::norm(r2);
      cv::Mat RR(3,3,CV_64F);
      r1.copyTo(RR.col(0));
      r2.copyTo(RR.col(1));
      // r3 = r1 x r2
      RR.at<double>(0,2) = r1.at<double>(1,0)*r2.at<double>(2,0) -
        r1.at<double>(2,0)*r2.at<double>(1,0);
      RR.at<double>(1,2) = r1.at<double>(2,0)*r2.at<double>(0,0) -
        r1.at<double>(0,0)*r2.at<double>(2,0);
      RR.at<double>(2,2) = r1.at<double>(0,0)*r2.at<double>(1,0) -
        r1.at<double>(1,0)*r2.at<double>(0,0);
      *rvec = cv::Mat(3,1,CV_64F);
      cv::Rodrigues(RR, *rvec);
      *tvec = cv::Mat(3,1,CV_64F);
      H.col(2).copyTo(*tvec);
    }

    // convert opencv rvec, tvec to gtsam pose
    static gtsam::Pose3 to_gtsam(const cv::Mat &rvec, const cv::Mat &tvec) {
      gtsam::Vector tvec_gtsam = (gtsam::Vector(3) <<
                                  tvec.at<double>(0),
                                  tvec.at<double>(1),
                                  tvec.at<double>(2)).finished();
      gtsam::Pose3 p(gtsam::Rot3::rodriguez(
                       rvec.at<double>(0),
                       rvec.at<double>(1),
                       rvec.at<double>(2)), tvec_gtsam);
      return (p);
    }

    static cv::Mat
    undistort_points(const std::vector<cv::Point2d> &image_points, 
                     const cv::Mat &K,  const std::string &distModel,
                     const cv::Mat &D) {
      cv::Mat im(image_points);
      if (distModel == "plumb_bob" || distModel == "radtan") {
        cv::Mat im_undist;
        cv::undistortPoints(im, im_undist, K, D);
        return (im_undist);
      } else if (distModel == "equidistant") {
        cv::Mat im_undist;
        cv::fisheye::undistortPoints(im, im_undist, K, D);
        return (im_undist);
      } else {
        std::cout << "ERROR: invalid distortion model: " << distModel << std::endl;
      }
      return (im);
    }

    static void
    project_points(const std::vector<cv::Point3d> &wp,
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
        std::cout << "WARNING: unknown distort model: " << distModel << " using radtan!" << std::endl;
        cv::projectPoints(wp, rvec, tvec, K, D, *ip);
      }
    }

    static double
    reprojection_error(const std::vector<cv::Point3d> &wp,
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

    static cv::Mat to_tf(const cv::Mat &rvec, const cv::Mat &tvec) {
      cv::Mat tf = cv::Mat::eye(4, 4, CV_64F);
      cv::Mat R;
      cv::Rodrigues(rvec, R);
      R.copyTo(tf(cv::Rect(0,0,3,3)));
      tvec.copyTo(tf(cv::Rect(3,0,1,3)));
      return (tf);
    }

    static void to_rvec_tvec(const cv::Mat &tf, cv::Mat *rvec, cv::Mat *tvec) {
      *rvec = cv::Mat_<double>(3, 1, CV_64F);
      *tvec = cv::Mat_<double>(3, 1, CV_64F);
      cv::Rodrigues(tf(cv::Rect(0,0,3,3)), *rvec);
      tf(cv::Rect(3,0,1,3)).copyTo(*tvec);
    }

    static cv::Mat transform_points(const cv::Mat &tf, const cv::Mat &p3) {
      cv::Mat wph;
      cv::convertPointsToHomogeneous(p3, wph);
      const cv::Mat pts4 = cv::Mat(p3.rows, 4, CV_64F, wph.data);
      const cv::Mat wpt  = (tf(cv::Rect(0,0,4,3)) * pts4.t()).t();
      return (wpt);
    }

    static void find_pose_by_homography(const cv::Mat &wp, const cv::Mat &ip,
                                        const cv::Mat &T_w_o, cv::Mat *rvec,
                                        cv::Mat *tvec) {
      // transform world points to object coordinates of that particular tag
      cv::Mat op(transform_points(T_w_o.inv(), wp));
      // but the reference camera system is 1.0 away from the tag, so
      // we need to shift the coordinates all by 1.0 in the z direction
      op.col(2) = 1.0;
      cv::Mat H = cv::findHomography(op, ip);
       // rvo and tvo are from reference (not world!) to camera frame!
      cv::Mat rvo, tvo;
      decompose_homography(H, &rvo, &tvo);
      // T_r_o has the 1.0 z-shift from object to camera frame
      cv::Mat T_r_o = cv::Mat::eye(4,4, CV_64F);
      T_r_o.at<double>(2,3) = 1.0;
      // T_c_o = T_c_r * T_r_o
      //const cv::Mat T_c_o = to_tf(rvo, tvo) * T_r_o;
      const cv::Mat T_c_o = to_tf(rvo, tvo);
      const cv::Mat T_c_w = T_c_o * T_w_o.inv();
      to_rvec_tvec(T_c_w, rvec, tvec);
    }
    
    static bool has_negative_z(const cv::Mat &wp, const cv::Mat &rvec, const cv::Mat &tvec) {
      const cv::Mat T_c_w  = to_tf(rvec, tvec);
      cv::Mat wpt = transform_points(T_c_w, wp);
      double minVal;
      cv::minMaxLoc(wpt.col(2), &minVal);
      if (minVal <= 0) {
        return (true);
      }
      return (false);
    }
    

    PoseEstimate pnp(const std::vector<cv::Point3d> &world_points,
                     const std::vector<cv::Point2d> &image_points,
                     const std::vector<cv::Mat> &T_w_o,
                     const cv::Mat &K,
                     const std::string &distModel,
                     const cv::Mat &D) {
      const cv::Mat im(image_points);
      const cv::Mat wp(world_points);
      const cv::Mat imu = undistort_points(image_points, K, distModel, D);
      //
      // loop over points in blocks of 4. Each block corresponds to a tag
      //
      const cv::Mat eye = cv::Mat::eye(3, 3, CV_64F);
      const cv::Mat D0  = cv::Mat::zeros(0, 4, CV_64F);
      double errMin = 1e10;
      cv::Mat rvecMin, tvecMin;
      for (const auto t_idx: irange(0ul, T_w_o.size())) {
        const auto i = 4 * t_idx;
        cv::Mat rvec, tvec;
        const cv::Mat im_tag(im(cv::Rect(0,i,1,4)));
        const cv::Mat wp_tag(wp(cv::Rect(0,i,1,4)));
        const cv::Mat imu_tag(imu(cv::Rect(0,i,1,4)));
        // first try to get an initial pose from the i'th tag
        bool status = cv::solvePnP(wp_tag, imu_tag, eye, D0,
                                   rvec, tvec, false, CV_P3P);
        if (!status || has_negative_z(wp, rvec, tvec)) {
          find_pose_by_homography(wp_tag, imu_tag,
                                  T_w_o[t_idx], &rvec, &tvec);
          status = !has_negative_z(wp, rvec, tvec);
        }
        if (status) {
          // yes, we got a pose, now lets use that as a starting
          // point for iterative optimization over all points.
          status = cv::solvePnP(wp, imu, eye, D0,
                                rvec,   tvec, true, CV_ITERATIVE);
          if (status && !has_negative_z(wp, rvec, tvec)) {
            // compute projection error
            double err = reprojection_error(world_points, image_points,
                                            rvec, tvec, K, distModel, D);
            if (err < errMin) {
              errMin = err;
              rvecMin = rvec;
              tvecMin = tvec;
            }
          }
        }
      }

      if (errMin < 1e10) {
        const auto T_c_w = to_gtsam(rvecMin, tvecMin);
        return (PoseEstimate(T_c_w.inverse(), errMin, 0));
      }
      return (PoseEstimate());
    }
  }
}
