#ifndef QUADRATICINTERCEPT_H
#define QUADRATICINTERCEPT_H

#include <QMap>

#include "linearplanner3d.h"
#include "path.h"
#include "quadstate.h"
#include "quadratic3d.h"
#include "simplequadraticintercept.h"

class QuadraticIntercept {
public:
  QuadraticIntercept();

  // target trajectory, where T = 0 is the current time
  Quadratic3d target_;

  // current quadcopter state
  QuadState state_;

  // Calculate the intercept path for the given target, under the various acceleration constraints.
  // The pointer found is set to true or false depending on whether a valid path was found. If no
  // trajectory was found, then will return the trajectory to the current target_ position
  // (at t=0).
  shared_ptr<Path> interceptPath(double T_hint, bool* found, LinearPlanner3d* simpleIntercept) const;
};

class CompletePath : public Path {
public:
  CompletePath(shared_ptr<Path> original, QuadState const& quad_state);

  Vector3d position(double t) const;
  Vector3d velocity(double t) const;
  double duration() const { return original_->duration(); }
  Vector3d initialAccelerationDirection() const;

private:
  shared_ptr<Path> original_;
  QuadState quad_state_;
};

#endif // QUADRATICINTERCEPT_H
