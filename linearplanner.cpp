#include "linearplanner.h"

#include <algorithm>
#include <cassert>
#include <cmath>

LinearPlanner::LinearPlanner() {
  x0 = 0;
  v0 = 0;
  x1 = 0;
  v1 = 0;
  time_step = 0;
  max_accel = 0;
}

double abs(double x) {
  return x > 0 ? x : -x;
}

void LinearPlanner::calcNext(double* x, double* v) const {
  // Aim to match speed and velocity, by accelerating in one direction, then the other.
  // Acclerate in one direction for time t0, then accelerate in other for time t1.
  // So solve for:
  // v0 + a*t0 - a*t1 = v1
  // x0 + v0*(t0+t1) + a*t0^2/2 + a*t0*t1 - a*t1^2/2 = x1 + v1*(t0+t1)

  double dv = v1 - v0;
  double dx = x1 - x0;
  int sign;
  if (dx == 0) {
    sign = dv > 0 ? +1 : -1;
  } else if (dv == 0 || dv * dx > 0 || dv*dv/2 <= max_accel * abs(dx)) {
    sign = dx > 0 ? +1 : -1;
  } else {
    sign = dx > 0 ? -1 : +1;
  }
  
  double a = sign * max_accel;
  
  double t0 = (dv + sign * sqrt(0.5*dv*dv + a*dx))/a;
  assert(std::isfinite(t0));
  assert(t0 >= -1e-5);
  double t1 = -dv/a + t0;
  assert(t1 >= -1e-5);

  double s0 = std::min(t0, time_step);
  double s1 = std::min(t1, time_step-s0);
  double s2 = time_step-s0-s1;

  *v = v0 + a*s0 - a*s1;
  *x = x0 + v0*(s0+s1) + 0.5*a*s0*s0 + a*s0*s1 - 0.5*a*s1*s1 + (*v) * s2;
}
