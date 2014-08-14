#ifndef HIGHERCONTROLLER_H
#define HIGHERCONTROLLER_H

#include <memory>
using namespace std;

#include <Eigen/Geometry>
using namespace Eigen;

#include <QObject>

class Controller;
class LinearPlanner3d;
class Path;
class Quad;
class Quadratic3d;
class QuadState;
class SimpleQuadraticIntercept;

class HigherController {
public:
  HigherController();

  /**
   * Calculate the necessary propeller inputs given the current quad state.
   */
  Vector4d getPropInputs(const Quad* quad);

private:
  Quadratic3d stateTargetToQuadratic() const;
  shared_ptr< Path > interceptForTarget(const QuadState& state, LinearPlanner3d* simpleIntercept) const;

  Vector3d target_pos_;
  Controller *controller_;
  double prev_intercept_duration_;
};

#endif // HIGHERCONTROLLER_H
