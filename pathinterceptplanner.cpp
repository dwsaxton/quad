#include "pathinterceptplanner.h"

#include <cmath>
#include <iostream>
using namespace std;

#include "linearplanner3d.h"
#include "simplequadraticintercept.h"
#include "quad.h"

double newtonSearch(std::function< double (double)> fn, double x, bool* found) {
  // TODO make these parameters of this function?
  int maxIt = 40;
  double eps = 1e-8;
  double eps_deriv = 1e-5;
  for (int i = 0; i < maxIt; ++i) {
    double value = fn(x);
    if (abs(value) < eps) {
      *found = true;
      return x;
    }
    double derivative = (fn(x + eps_deriv) - value) / eps_deriv;
    double prev = x;
    x -= value / derivative;
    if (x < 0) {
      x = prev / 2.0;
    } else if (!isfinite(x) || x > 1e6) {
      x = 1e6;
    }
  }

  *found = false;
  return x;
}

double binarySearch(std::function<double (double)> fn, double left, double right, bool* found) {
  // TODO make these parameters of this function?
  int maxIt = 40;
  double eps = 1e-6;

  double valueLeft = fn(left);
  double valueRight = fn(right);

  if ((valueLeft > 0 && valueRight > 0) || (valueLeft < 0 && valueRight < 0)) {
    *found = false;
    return 0;
  }

  for (int i = 0; i < maxIt; ++i) {
    double mid = (left + right) / 2;
    double valueMid = fn(mid);

    if (abs(valueMid) < eps) {
      *found = true;
      return mid;
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
  return (left + right) / 2;
}

PathInterceptPlanner::PathInterceptPlanner() {
}

void TestInterceptPoint(PathInterceptPlanner *planner, Quadratic3d const& target) {
  planner->target_ = target;
  bool found;
  shared_ptr<Path> path = planner->interceptPath(1, &found);
  assert(found);
  double duration = path->duration();
  Vector3d pos = path->position(duration);
  Vector3d target_pos = target.eval(duration);
  Vector3d vel = path->velocity(duration);
  Vector3d target_vel = target.derivative().eval(duration);
  assert((pos - target_pos).norm() < 1e-3);
  assert((vel - target_vel).norm() < 1e-3);
}

void TestPathInterceptPlanner(PathInterceptPlanner *planner) {
  TestInterceptPoint(planner, Quadratic3d({0, 0, 5}, {0, 0, 1}, {-1, 14, 14}));
}

void TestPathInterceptPlanners() {
  LinearPlanner3d linear_planner;
//   SimpleQuadraticIntercept quadratic_planner;
  TestPathInterceptPlanner(&linear_planner);
//   TestPathInterceptPlanner(&quadratic_planner);
}
