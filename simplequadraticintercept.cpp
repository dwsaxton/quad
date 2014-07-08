#include "simplequadraticintercept.h"

#include <cmath>
#include <iostream>
using namespace std;


SimpleQuadraticIntercept::SimpleQuadraticIntercept() {
  max_linear_acceleration_ = 12; // slightly faster than gravity for default value
  max_pitch_acceleration_ = 2; // sensible default value too
}

Quadratic3d SimpleQuadraticIntercept::interceptPath(bool * found) const {
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
    *found = false;
    return calcInterceptPathForT(0);
  }

  double valueRight = f(right);

  for (int i = 0; i < maxIt; ++i) {
    double mid = (left + right) / 2;
    double valueMid = f(mid);

    if (abs(valueMid) < eps) {
      *found = true;
      cout << "intercept at T="<<mid<<endl;
      return calcInterceptPathForT(valueMid);
    }

    if ((valueLeft > 0) == (valueMid > 0)) {
      left = mid;
      valueLeft = valueMid;
    } else {
      right = mid;
      valueRight = valueMid;
    }
  }

  *found = false;
  return calcInterceptPathForT(0);
}

Quadratic3d SimpleQuadraticIntercept::calcInterceptPathForT(double T) const {
  Vector3d pos = target_.eval(T);
  Vector3d deriv = target_.derivative().eval(T);
  Quadratic3d traj;
  traj.a = deriv - pos;
  traj.b = 2*pos - deriv;
  return traj;
}

double SimpleQuadraticIntercept::f(double T) const {
  // target_ interception speed
  double v = target_.derivative().eval(T).norm();
  Quadratic3d intercept = calcInterceptPathForT(T);

  // calculate the time it takes to orientate ourselves in the direction of the path
  Vector3d initial_direction = intercept.b.normalized();
  double angle = acos(initial_direction.z());
  double angle_threshold = M_PI / 4; // TODO should this be standardized somewhere?
  double rotation_required = max(0.0, angle - angle_threshold);
  double rotation_time = 2 * sqrt(rotation_required / max_pitch_acceleration_);

  return 0.5*v*v/max_linear_acceleration_ + v*(T - v/max_linear_acceleration_ - rotation_time) - intercept.length(1);
}