#include "linearplannermono.h"

#include <cassert>
#include <cmath>
#include <list>
#include <iostream>
using namespace std;

LinearPlannerMono::LinearPlannerMono() {
  x1 = v1 = t0 = t1 = t2 = a = 0;
}

double LinearPlannerMono::getMinDuration(double max_accel) const {
  LinearPlannerMono temp(*this);
  temp.setupForMaxAccel(max_accel);
  return temp.duration();
}

void LinearPlannerMono::setupForDuration(double T) {
  // We have t1 = v / a.
  // The total duration is T = t0 + t1 + t2
  // x = v^2 / (2*a) + v t2
  // The constraint t2 >= 0 implies that a >= v^2 / 2x
  // The constraint t1 <= T implies that a >= v / T
  // The constraint t0 >= 0 implies that a >= v^2 / 2(Tv-x)
  // Other than these constraints, we take a to be as small as possible,
  // and calculate the necessary values of t0, t1 and t2

  a = max(v1 == 0 ? 0 : v1 * v1 / 2 / (T * v1 - x1), max(v1 / T, v1 == 0 ? 0 : v1 * v1 / 2 / x1));
  t1 = v1 == 0 ? 0 : v1 / a;
  t2 = (x1 == 0 ? 0 : x1 / v1) - t1 / 2;
  t0 = T - t1 - t2;
}

void LinearPlannerMono::setupForMaxAccel(double max_accel) {
  a = max_accel;
  t0 = 0;
  t1 = v1 / max_accel;
  t2 = (x1 == 0 ? 0 : x1 / v1) - t1 / 2;
}

bool LinearPlannerMono::isValid(double *penalty) const {
  if (x1 < 0 || v1 < 0 || (v1 == 0 && x1 != 0) || t0 < 0 || t1 < 0 || t2 < 0
      || !isfinite(t0) || !isfinite(t1) || !isfinite(t2) || !isfinite(a)) {
    if (penalty) {
      assert(max(1.2, 1.4) == 1.4); // using double max
      *penalty = max(0., -x1) + max(0., -v1) + max(0., -t0) + max(0., -t1) + max(0., -t2);
    }
    return false;
  }
  if (penalty) {
    *penalty = 0;
  }
  return true;
}

void LinearPlannerMono::getPosVel(double time, double *x, double *v) const {
  if (time > duration() + 1e-6) {
//     cout << "LinearPlannerMono::getPosVel: WARNING: requesting time " << time << " beyond provisioned duration of " << duration() << endl;
  }

  double s0 = std::min(t0, time);
  double s1 = std::min(t1, time - s0);
  double s2 = time - s0 - s1;

  *x = 0.5 * a * s1 * s1 + a * s1 * s2;
  *v = s1 * a;
}
