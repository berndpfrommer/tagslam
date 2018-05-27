/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_POSITION_MEASUREMENT_H
#define TAGSLAM_POSITION_MEASUREMENT_H

#include <vector>
#include <memory>
#include <ros/ros.h> // xmlrpc
#include <gtsam/geometry/Point3.h>

namespace tagslam {
  struct PositionMeasurement {
    PositionMeasurement(const std::string &n  = std::string(""),
                        int t = 0, int c = 0, const gtsam::Point3 &dr =
                        gtsam::Point3(1.0, 0.0, 0.0),
                        double l = 0, double nd = 0) :
      name(n), tag(t), corner(c), dir(dr), length(l), noise(nd) {};
    // -------------------------
    typedef std::shared_ptr<PositionMeasurement> PositionMeasurementPtr;
    typedef std::shared_ptr<const PositionMeasurement> PositionMeasurementConstPtr;
    typedef std::vector<PositionMeasurementPtr> PositionMeasurementVec;
    std::string         name;
    int                 tag;
    int                 corner;
    gtsam::Point3       dir;
    double              length;
    double              noise;
    // -------- static functions
    static PositionMeasurementPtr parse(const std::string &name,
                                        XmlRpc::XmlRpcValue meas);
    static PositionMeasurementVec parse(XmlRpc::XmlRpcValue measurements);
  };

  using PositionMeasurementPtr = PositionMeasurement::PositionMeasurementPtr;
  using PositionMeasurementConstPtr = PositionMeasurement::PositionMeasurementConstPtr;
  using PositionMeasurementVec = PositionMeasurement::PositionMeasurementVec;
}

#endif
