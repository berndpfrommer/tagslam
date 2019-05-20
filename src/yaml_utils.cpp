/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/yaml_utils.h"
#include <boost/range/irange.hpp>
#include <XmlRpcException.h>
#include <Eigen/Geometry>

namespace tagslam {
  using boost::irange;

  static double read_field(XmlRpc::XmlRpcValue v) {
    double x = 0;
    try {
      x = static_cast<double>(v);
    } catch (const XmlRpc::XmlRpcException &e) {
      x = (double)static_cast<int>(v);
    }
    return (x);
  }
  namespace yaml_utils {
    Eigen::Vector3d get_vec(const std::string &name,
                            XmlRpc::XmlRpcValue v) {
      try {
        double x(0), y(0), z(0);
        for (XmlRpc::XmlRpcValue::iterator it = v.begin();
             it != v.end(); ++it) {
          std::string field = it->first;
          if (field == "x") {        x = read_field(it->second);
          } else if (field == "y") { y = read_field(it->second);
          } else if (field == "z") { z = read_field(it->second);
          }
        }
        return (Eigen::Vector3d(x, y, z));
      } catch (const XmlRpc::XmlRpcException &e) {
        throw std::runtime_error("error parsing vector: " + name);
      }
    }

    static void write_vec(std::ostream &of,
                          const std::string &prefix,
                          double x, double y, double z) {
      const int p(8);
      of.precision(p);
      of << prefix << "x: " << std::fixed << x << std::endl;
      of << prefix << "y: " << std::fixed << y << std::endl;
      of << prefix << "z: " << std::fixed << z << std::endl;
    }


    bool get_pose_and_noise(XmlRpc::XmlRpcValue pose_and_noise,
                            Transform *pose, PoseNoise2 *noise,
                            double defPosNoise, double defRotNoise) {
      Eigen::Vector3d anglevec, center;
      Point3d rotnoise, posnoise;
      int nfound(0);
      bool foundRotNoise(false), foundPosNoise(false);
      for (XmlRpc::XmlRpcValue::iterator it = pose_and_noise.begin();
           it != pose_and_noise.end(); ++it) {
        if (it->first == "rotation") {
          anglevec = get_vec("rotation", it->second);
          nfound++;
        } else if (it->first == "position") {
          center   = get_vec("position", it->second);
          nfound++;
        } else if (it->first == "rotation_noise") {
          rotnoise = get_vec("rotation_noise", it->second);
          foundRotNoise = true;
        } else if (it->first == "position_noise") {
          posnoise = get_vec("position_noise", it->second);
          foundPosNoise = true;
        }
      }
      
      *pose = make_transform(utils::rotmat_eigen(anglevec), center);
      if (!foundRotNoise) {
        rotnoise = Point3d();
        rotnoise << defRotNoise, defRotNoise, defRotNoise;
      }
      if (!foundPosNoise) {
        posnoise = Point3d();
        posnoise << defPosNoise, defPosNoise, defPosNoise;
      }
      *noise = PoseNoise2::make(rotnoise, posnoise);
      return (nfound == 2);
    }
 
    void write_pose(std::ostream &of, const std::string &prefix,
                    const Transform &pose,
                    const PoseNoise2 &n, bool writeNoise) {
      Eigen::AngleAxisd aa(pose.linear());
      Point3d r = aa.angle() * aa.axis();
      Point3d t(pose.translation());
      const std::string pps = prefix + "  ";
      of << prefix << "position:" << std::endl;
      write_vec(of, pps, t(0), t(1), t(2));
      of << prefix << "rotation:" << std::endl;
      write_vec(of, pps, r(0), r(1), r(2));
      if (writeNoise) {
        Eigen::Matrix<double, 6, 1> diag = n.getDiagonal();
        of << prefix << "position_noise:" << std::endl;
        write_vec(of, pps, diag(3),diag(4),diag(5));
        of << prefix << "rotation_noise:" << std::endl;
        write_vec(of, pps, diag(0),diag(1),diag(2));
      }
    }

    void write_pose_with_covariance(std::ostream &of,
                                    const std::string &prefix,
                                    const Transform &pose,
                                    const PoseNoise2 &n) {
      Eigen::AngleAxisd aa;
      aa.fromRotationMatrix(pose.rotation());
      Eigen::Vector3d r = aa.angle() * aa.axis();
      Eigen::Vector3d t = pose.translation();
      const std::string pps = prefix + "  ";
      of << prefix << "position:" << std::endl;
      write_vec(of, pps, t(0), t(1), t(2));
      of << prefix << "rotation:" << std::endl;
      write_vec(of, pps, r(0), r(1), r(2));
      of << prefix << "R:" << std::endl;
      const auto R = n.convertToR();
      of << prefix << "  [ ";
      for (const auto i: irange(0l, R.rows())) {
        for (const auto j: irange(0l, R.cols())) {
          of << R(i, j);
          if (i != R.rows() - 1 || j != R.cols() - 1) {
            of << ", ";
          }
        }
        if (i != R.rows() - 1) {
          of << std::endl << prefix << "    ";
        }
      }
      of << " ]" << std::endl;
    }
  }
}  // namespace
