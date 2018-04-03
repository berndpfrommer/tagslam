/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_DISTANCE_MEASUREMENT_H
#define TAGSLAM_DISTANCE_MEASUREMENT_H

#include <vector>
#include <memory>
#include <ros/ros.h> // xmlrpc

namespace tagslam {
  struct DistanceMeasurement {
    DistanceMeasurement(const std::string &n  = std::string(""),
                        int t1 = 0, int c1 = 0, int t2 = 0, int c2 = 0,
                        double d = 0, double nd = 0) :
      name(n), tag1(t1), corner1(c1), tag2(t2), corner2(c2),
      distance(d), noise(nd) {};
    // -------------------------
    typedef std::shared_ptr<DistanceMeasurement> DistanceMeasurementPtr;
    typedef std::shared_ptr<const DistanceMeasurement> DistanceMeasurementConstPtr;
    typedef std::vector<DistanceMeasurementPtr> DistanceMeasurementVec;
    std::string         name;
    int                 tag1;
    int                 corner1;
    int                 tag2;
    int                 corner2;
    double              distance;
    double              noise;
    // -------- static functions
    static DistanceMeasurementPtr parse(const std::string &name,
                                        XmlRpc::XmlRpcValue meas);
    static DistanceMeasurementVec parse(XmlRpc::XmlRpcValue measurements);
  };

  using DistanceMeasurementPtr = DistanceMeasurement::DistanceMeasurementPtr;
  using DistanceMeasurementConstPtr = DistanceMeasurement::DistanceMeasurementConstPtr;
  using DistanceMeasurementVec = DistanceMeasurement::DistanceMeasurementVec;
}

#endif
