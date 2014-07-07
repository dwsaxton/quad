#ifndef QUADRATICINTERCEPT_H
#define QUADRATICINTERCEPT_H

#include "quadstate.h"
#include "quadratic3d.h"

class QuadraticIntercept {
public:
  QuadraticIntercept();

  // target trajectory, where T = 0 is the current time
  Quadratic3d target_;

  // maximum linear acceleration
  double max_linear_acceleration_;

  // maximum pitch or roll acceleration (yaw acceleration will be different, but we don't care
  // about this) in radians per second per second.
  double max_pitch_acceleration_;

  // current quadcopter state
  QuadState state_;

  // Calculate the intercept path for the given target, under the various acceleration constraints.
  // The pointer found is set to true or false depending on whether a valid path was found. If no
  // trajectory was found, then will return the trajectory to the current target_ position
  // (at t=0).
  Quadratic3d interceptPath(bool * found) const;

};

#endif // QUADRATICINTERCEPT_H
