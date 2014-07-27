#include "linearplanner.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
using namespace std;

#include "pathinterceptplanner.h"

LinearPlanner::LinearPlanner() {
  x0 = v0 = x1 = v1 = t0 = t1 = a = 0;
}

void LinearPlanner::setInitial(double x, double v) {
  x0 = x;
  v0 = v;
}

void LinearPlanner::setTarget(double x, double v) {
  x1 = x;
  v1 = v;
}

double LinearPlanner::getMinDuration(double max_accel) const {
  LinearPlanner temp(*this);
  temp.setupForMaxAccel(max_accel);
  return temp.duration();
}


void LinearPlanner::setupForMaxAccel(double max_accel) {
  double dv = v1 - v0;
  double dx = x1 - x0;

  if (abs(dv) < 1e-6 && abs(dx) < 1e-6) {
    t0 = t1 = 0;
    a = max_accel;
    return;
  }

  if (max_accel < 1e-10) {
    cout << "LinearPlanner::setupForMaxAccel: WARNING: max accel too small or negative" << endl;
    return;
  }

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

//   // TODO work out which root we should take analytically?
  for (int s = -1; s <= +1; s += 2) {
    a = s * max_accel;
    t0 = (-v0 + s * sqrt(0.5*v0*v0 + 0.5*v1*v1 + a*dx))/a;
    t1 = -dv / a + t0;
    if (!std::isfinite(t0) || t0 < -1e-5 || t1 < -1e-5) {
      continue;
    }
    break;
  }
  assert(std::isfinite(t0));
  assert(t0 >= -1e-5);
  assert(t1 >= -1e-5);
}

void LinearPlanner::setupForDuration(double T) {
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
    a = aplus;
    double dv_a = dv == 0 ? 0 : dv / a;
    t0 = (T + dv_a) / 2;
    t1 = (T - dv_a) / 2;
    if (t0 >= 0 && t1 >= 0) {
      double x, v;
      getPosVel(T, &x, &v);
      if (isValid() && abs(x - x1) < 1e-4 && abs(v - v1) < 1e-4) {
        return;
      }
    }
  }
  if (isfinite(aminus)) {
    a = aminus;
    double dv_a = dv == 0 ? 0 : dv / a;
    t0 = (T + dv_a) / 2;
    t1 = (T - dv_a) / 2;
    if (t0 >= 0 && t1 >= 0) {
      double x, v;
      getPosVel(T, &x, &v);
      if (isValid() && abs(x - x1) < 1e-4 && abs(v - v1) < 1e-4) {
        return;
      }
    }
  }
  cout << "LinearPlanner::setupForDuration: WARNING: could not setup for duration " << T << endl;
}

void LinearPlanner::getPosVel(double time, double* x, double* v) const {
  if (time > duration() + 1e-6) {
//     cout << "LinearPlannerMono::getPosVel: WARNING: requesting time " << time << " beyond provisioned duration of " << duration() << endl;
  }

  double s0 = std::min(t0, time);
  double s1 = std::min(t1, time - s0);
  double s2 = time - s0 - s1;

  *v = v0 + a*s0 - a*s1;
  *x = x0 + v0*(s0+s1) + 0.5*a*s0*s0 + a*s0*s1 - 0.5*a*s1*s1 + (*v) * s2;
}
