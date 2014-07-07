#ifndef SIMPLEQUADRATICINTERCEPT_H
#define SIMPLEQUADRATICINTERCEPT_H

#include "quadratic3d.h"

/**
 * Calculate an intercept to a quadratic trajectory, given that we are currently pointing upwards.
 * Simple because we assume we start from stationary from the origin and are pointing upwards.
 * This is used by the more advanced QuadraticIntercept class which has no assumptions on the
 * origin.
 */
class SimpleQuadraticIntercept {
public:
  SimpleQuadraticIntercept();

  // target trajectory, where T = 0 is the current time
  Quadratic3d target_;

  // maximum linear acceleration
  double max_linear_acceleration_;

  // maximum pitch or roll acceleration (yaw acceleration will be different, but we don't care
  // about this) in radians per second per second.
  double max_pitch_acceleration_;

  // Calculate the intercept path for the given target, under the various acceleration constraints.
  // The pointer found is set to true or false depending on whether a valid path was found. If no
  // trajectory was found, then will return the trajectory to the current target_ position
  // (at t=0).
  Quadratic3d interceptPath(bool * found) const;

private:
  // Calculate the quadratic q(x) such that q(0) = 0, q(1) = target(T), and q'(1) = target'(T).
  Quadratic3d calcInterceptPathForT(double T) const;

  // return the distance travelled by time T along the intercept path given by calcInterceptPath(T),
  // and the actual length of this intercept path
  double f(double T) const;
};

#endif // SIMPLEQUADRATICINTERCEPT_H

