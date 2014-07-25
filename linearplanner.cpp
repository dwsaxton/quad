#include "linearplanner.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
using namespace std;

#include "pathinterceptplanner.h"

LinearPlanner::LinearPlanner() {
  params_changed_ = true;
  x0 = 0;
  v0 = 0;
  x1 = 0;
  v1 = 0;
  max_accel_ = 0;
  t0 = 0;
  t1 = 0;
  a = 0;
}

void LinearPlanner::setInitial(double x, double v) {
  x0 = x;
  v0 = v;
  params_changed_ = true;
}

double LinearPlanner::setTarget(double x, double v) {
  x1 = x;
  v1 = v;
  params_changed_ = true;
}
  
double LinearPlanner::setMaxAccel(double max_accel) {
  max_accel_ = max_accel;
  params_changed_ = true;
}

// double abs(double x) {
//   return x > 0 ? x : -x;
// }

// #if 0
int sign(double dx, double dv, double max_accel) {
  if (dx == 0) {
    return dv > 0 ? +1 : -1;
  } else if (dv == 0 || dv * dx > 0 || dv*dv/2 <= max_accel * std::abs(dx)) {
    return dx > 0 ? +1 : -1;
  } else {
    return dx > 0 ? -1 : +1;
  }
}
// #endif

void LinearPlanner::plan() const {
  if (std::abs(max_accel_) < 1e-10) {
    return;
  }
  if (!params_changed_) {
    return;
  }
  params_changed_ = false;

  // Aim to match speed and velocity, by accelerating in one direction, then the other.
  // Acclerate in one direction for time t0, then accelerate in other for time t1.
  // So solve for:
  // dv = a*t0 - a*t1
  // dx = v0*(t0+t1) + a*t0^2/2 + a*t0*t1 - a*t1^2/2
  // Solution:
  // t0 =   (-v0 +-   sqrt(v0^2 / 2 + v1^2 / 2 + a dx)) / a
  // t1 = t0 - dv / a
  //    =   (-v1 +-   sqrt(v0^2 / 2 + v1^2 / 2 + a dx)) / a
  // T = (-v0-v1 +- 2 sqrt(v0^2 / 2 + v1^2 / 2 + a dx)) / a

  double dv = v1 - v0;
  double dx = x1 - x0;
//   // TODO work out which root we should take analytically?
  for (int s = -1; s <= +1; s += 2) {
//   int s = sign(dx, dv, max_accel_);
    a = s * max_accel_;
    t0 = (-v0 + s * sqrt(0.5*v0*v0 + 0.5*v1*v1 + a*dx))/a;
    t1 = -dv/a + t0;
    if (!std::isfinite(t0) || t0 < -1e-5 || t1 < -1e-5) {
      continue;
    }
    break;
  }
  assert(std::isfinite(t0));
  assert(t0 >= -1e-5);
  assert(t1 >= -1e-5);
}

void LinearPlanner::updateMaxAccelForDuration(double T) {
  if (T == 0) {
    return;
  }
  
  double dv = v1 - v0;
  double dx = x1 - x0;
  // solve for T^2 a^2  + (2 T v0 + 2 T v1 - 4 dx) a - dv^2 = 0
  double qa = T * T;
  double qb = 2 * T * v0 + 2 * T * v1 - 4 * dx;
  double qc = - dv * dv;
  double aplus  = (-qb + sqrt(qb * qb - 4 * qa * qc)) / (2 * qa);
  double aminus = (-qb - sqrt(qb * qb - 4 * qa * qc)) / (2 * qa);
  
  if (isfinite(aplus)) {
    setMaxAccel(abs(aplus));
    if (abs(duration() - T) < 1e-4) {
      double x, v;
      getPosVel(T, &x, &v);
      if (abs(x - x1) < 1e-4 && abs(v - v1) < 1e-4) {
        return;
      }
    }
  }
  if (isfinite(aminus)) {
    setMaxAccel(abs(aminus));
    if (abs(duration() - T) < 1e-4) {
      double x, v;
      getPosVel(T, &x, &v);
      if (abs(x - x1) < 1e-4 && abs(v - v1) < 1e-4) {
        return;
      }
    }
  }
//   cout << "WARNING non matching" << endl;
}

void LinearPlanner::getPosVel(double time, double* x, double* v) const {
  plan();

  double s0 = std::min(t0, time);
  double s1 = std::min(t1, time - s0);
  double s2 = time - s0 - s1;

  *v = v0 + a*s0 - a*s1;
  *x = x0 + v0*(s0+s1) + 0.5*a*s0*s0 + a*s0*s1 - 0.5*a*s1*s1 + (*v) * s2;
}

double LinearPlanner::duration() const {
  plan();
  return t0 + t1;
}