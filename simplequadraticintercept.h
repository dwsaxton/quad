#ifndef SIMPLEQUADRATICINTERCEPT_H
#define SIMPLEQUADRATICINTERCEPT_H

#include <QMap>

#include "path.h"
#include "quadratic3d.h"

class PathInfo {
public:
  PathInfo() {
    rotation_duration = 0;
    accel_duration = 0;
    length = 0;
    duration = 0;
    valid = false;
  }

  double accelDurationAtTime(double t) const;
  double lengthTraveled(double t, double accel) const;

  Quadratic3d quadratic; // the path to follow
  double rotation_duration; // in seconds, time to initially rotate to the correct orientation
  double accel_duration; // in seconds, time to apply acceleration, after the rotation period has finished
  double length; // in meters, the length before intercept is achieved
  double duration; // in seconds, the total time before we intercept
  bool valid; // whether the path is actually valid (did the algorithm find a path?)
};

// TODO put this in a separate math util class?
double paramForZero(std::function<double (double)> fn, double left, double right, bool *found);

/**
 * Calculate an intercept to a quadratic trajectory, given that we are currently pointing upwards.
 * Simple because we assume we start from stationary from the origin and are pointing upwards.
 * This is used by the more advanced QuadraticIntercept class which has no assumptions on the
 * origin, nor on a uniform acceleration field (that's gravity).
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
  // (at t=0). The accel_duration is how long we should accelerate for max_linear_acceleration_.
  // The length is how long the intercept path is.
  Path *interceptPath(bool* found) const;

private:
  // Calculate the quadratic q(x) such that q(0) = 0, q(1) = target(T), and q'(1) = target'(T).
  PathInfo calcInterceptPathForT(double T) const;

  // return the distance travelled by time T along the intercept path given by calcInterceptPath(T),
  // and the actual length of this intercept path
  double f(double T, PathInfo *info = nullptr) const;

  double targetSpeed(double T) const;
};

class PathFromQuadratic : public Path {
public:
  PathFromQuadratic(const PathInfo& info, double accel);

  Vector3d position(double t) const;
  Vector3d velocity(double t) const;
  double duration() const { return info_.duration; }
  Vector3d initialAccelerationDirection() const { return info_.quadratic.derivative().eval(0).normalized(); }

private:
  double parameterForTime(double t) const;

  mutable QMap<double, double> paramater_for_time_cache_;

  PathInfo info_;
  double accel_;
};

#endif // SIMPLEQUADRATICINTERCEPT_H

