//-*-C++-
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/expressions.h>

#include <vector>

using namespace std;
using namespace gtsam;

#define USE_EXPRESSION_FACTORS

int main(int argc, char* argv[]) {
  const double pixNoise = std::sqrt(std::atof(argv[1]));
  // focal length = 1.0, image center = 0,0
  Cal3_S2::shared_ptr K(new Cal3_S2(1.0, 1.0, 0.0, 0.0, 0.0));
  const auto measNoise = noiseModel::Isotropic::Sigma(2, pixNoise);
  const double sp = 1.0; // physical size of tag (edge length)
  const double Z  = 1.0; // distance of camera to tag
  // set of ground-truth landmarks: 4 corners in the x-y plane
  const vector<Point3> p3d = { Point3(-sp/2, -sp/2, 0),
                               Point3( sp/2, -sp/2, 0),
                               Point3( sp/2,  sp/2, 0),
                               Point3(-sp/2,  sp/2, 0)};
  ExpressionFactorGraph graph;
  Values initVals;
  // the tag is in the x-y plane, facing up along the z-direction
  // camera pose is rotated along x such that it faces the tag at distance Z
  const Pose3 camPose(Rot3(0, 1.0, 0, 0), Point3(0, 0, Z));
  const SimpleCamera camera(camPose, *K);
  const Symbol camPoseSymbol('x', 0);
  initVals.insert(Symbol('x', 0), camPose); // init to correct value

  for (size_t i = 0; i < p3d.size(); i++) { // loop over 4 corners
    Point2 meas = camera.project(p3d[i]); // projected camera point
    std::cout << "meas: " << meas << std::endl;
#ifdef USE_EXPRESSION_FACTORS
    //
    Expression<Pose3>  T_c_w(camPoseSymbol);   // world-to-camera transform
    Expression<Point3> X_o(p3d[i]);
    Expression<Point2> projPt = gtsam::project(transformFrom(T_c_w, X_o));
    gtsam::Expression<Cal3_S2> cK(*K);
    Expression<Point2> predict(cK, &Cal3_S2::uncalibrate, projPt);
    graph.addExpressionFactor(predict, meas, measNoise);
#else

    graph.push_back(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
                      meas, measNoise, camPoseSymbol, Symbol('l', i), K));
    // fix landmarks to locations via strong prior, shouldn't move
    auto lmNoise = noiseModel::Isotropic::Sigma(3, 1e-7);
    graph.push_back(PriorFactor<Point3>(Symbol('l', i), p3d[i], lmNoise));
    // initialize landmarks perfect
    initVals.insert(Symbol('l', i), p3d[i]);
#endif    
  }
  //graph.print("Factor Graph:\n");
  Values result = DoglegOptimizer(graph, initVals).optimize();
  //result.print("Final results:\n");
  Marginals marginals(graph, result);
  marginals.print();
  std::cout << "camera pose: " << std::endl <<
    result.at<gtsam::Pose3>(camPoseSymbol) << std::endl;
  std::cout << "camera pose covariance: " << std::endl;
  std::cout << marginals.marginalCovariance(camPoseSymbol) << std::endl;
  return 0;
}
