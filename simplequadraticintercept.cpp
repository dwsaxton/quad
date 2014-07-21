#include "simplequadraticintercept.h"

#include <cmath>
#include <iostream>
using namespace std;

double PathInfo::accelDurationAtTime(double t) const {
  return max(0.0, min(t - rotation_duration, accel_duration));
}

double PathInfo::lengthTraveled(double t, double accel) const {
  double accel_t = accelDurationAtTime(t);
  double length = 0.5 * accel * accel_t * accel_t;
  double cutoff = rotation_duration + accel_duration;
  if (t > cutoff) {
    length += (t - cutoff) * accel * accel_duration;
  }
  return length;
}

Path* SimpleQuadraticIntercept::interceptPath(double T_hint, bool *found) const {
  // Do search for correct T
  double left = 0; // always search for positive time. (can't fly backwards in time!)

  int maxIt = 23;
  double eps = 1e-3; // 1 mm accuracy
  double maxT = min(1e6, eps * (1 << (maxIt-1)));

  // (find a good right value)
  double right = 10;
  double valueLeft = f(left);
  while ((f(right) > 0) == (valueLeft > 0) && right < maxT) {
    right *= 2;
  }
  if (right >= maxT) {
    return new PathFromQuadratic(calcInterceptPathForT(0), max_linear_acceleration_);
  }

  auto function = [&] (double x) { return f(x); };

  // TODO passing of eps, iteration parameters
  double T = newtonSearch(function, T_hint, found);
//   cout << "SimpleQuadraticIntercept::interceptPath: T: " << T << endl;
  PathInfo info = calcInterceptPathForT(T, true);
  return new PathFromQuadratic(info, max_linear_acceleration_);
}

PathInfo SimpleQuadraticIntercept::calcInterceptPathForT(double T, bool debug) const {
  Vector3d pos = target_.eval(T);
  Vector3d deriv = 5 * target_.derivative().eval(T);
  PathInfo info;
  info.duration = T;
  info.quadratic.a = deriv - pos;
  info.quadratic.b = 2*pos - deriv;

  Vector3d initial_direction = info.quadratic.b.normalized();
  double angle = acos(initial_direction.z());
  double angle_threshold = M_PI / 4; // TODO should this be standardized somewhere?
//   double angle_threshold = 0;
  double rotation_required = max(0.0, angle - angle_threshold);
  info.rotation_duration = rotation_required / max_pitch_acceleration_; // TODO this is not a good estimate for the length of time required for rotation
  info.length = info.quadratic.length(1);
  info.accel_duration = targetSpeed(T) / max_linear_acceleration_;

  if (debug) {
    cout << endl << "T: " << T << endl;
    cout << "pos(T): " << pos.transpose() << endl;
    cout << "deriv(T): " << deriv.transpose() << endl;
    cout << "angle: " << angle << endl;
    cout << "rotation_duration: " << info.rotation_duration << endl;
    cout << "length: " << info.length << endl;
    cout << "targetSpeed(T): " << targetSpeed(T) << endl;
    cout << "accel_duration: " << info.accel_duration << endl;
  }

  return info;
}

double SimpleQuadraticIntercept::targetSpeed(double T) const {
  return target_.derivative().eval(T).norm();
}

double SimpleQuadraticIntercept::f(double T, PathInfo *info) const {
  // target_ interception speed
  double v = targetSpeed(T);
  PathInfo temp;
  if (info == nullptr) {
    info = &temp;
  }
  *info = calcInterceptPathForT(T);

  return info->lengthTraveled(T, max_linear_acceleration_) - info->length;
}


PathFromQuadratic::PathFromQuadratic(const PathInfo& info, double accel) {
  info_ = info;
  accel_ = accel;
}

double PathFromQuadratic::parameterForTime(double t) const {
  if (t == 0) {
    return 0;
  }

  if (paramater_for_time_cache_.contains(t)) {
    return paramater_for_time_cache_.value(t);
  }
    
  double length_target = info_.lengthTraveled(t, accel_);
  auto length_error = [&] (double param) { return info_.quadratic.length(param) - length_target; };
  bool found;
  double param = binarySearch(length_error, 0, 1.1, &found);
  if (!found) {
    cout << "PathFromQuadratic::parameterForTime("<<t<<"): correct parameter not found" << endl;
  }
  paramater_for_time_cache_.insert(t, param);
  return param;
}

Vector3d PathFromQuadratic::position(double t) const {
  double param = parameterForTime(t);
  return info_.quadratic.eval(param);
}

Vector3d PathFromQuadratic::velocity(double t) const {
  double accel_duration = info_.accelDurationAtTime(t);
  double target_v = accel_ * accel_duration;
  double param = parameterForTime(t);
  Vector3d vel = info_.quadratic.derivative().eval(param);
  vel *= target_v / vel.norm();
  return vel;
}
