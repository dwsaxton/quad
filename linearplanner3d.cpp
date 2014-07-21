#include "linearplanner3d.h"

#include <iostream>
using namespace std;

Path* LinearPlanner3d::interceptPath(double T_hint, bool *found) const {
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
    return new LinearPlanner3dPath(calcInterceptPathForT(0));
  }

  auto function = [&] (double x) { return f(x); };

  // TODO passing of eps, iteration parameters
  double T = newtonSearch(function, T_hint, found);
  LinearPlanner3dPath path = calcInterceptPathForT(T);
  return new LinearPlanner3dPath(path); // TODO massive memory leak
}

LinearPlanner3dPath LinearPlanner3d::calcInterceptPathForT(double T) const {
  Vector3d pos = target_.eval(T);
  Vector3d vel = target_.derivative().eval(T);
  LinearPlanner3dPath path;
  for (int i = 0; i < 3; ++i) {
    path.planners_[i].setMaxAccel(max_linear_acceleration_); // TODO max accel individually in different coords?
    path.planners_[i].setTarget(pos.coeff(i), vel.coeff(i));
  }
  return path;
}

double LinearPlanner3d::f(double T) const {
  LinearPlanner3dPath path = calcInterceptPathForT(T);
  return path.duration() - T;
}

Vector3d LinearPlanner3dPath::position(double t) const {
  Vector3d pos;
  for (int i = 0; i < 3; ++i) {
    double x, v;
    planners_[i].getPosVel(t, &x, &v);
    pos(i) = x;
  }
  return pos;
}

Vector3d LinearPlanner3dPath::velocity(double t) const {
  Vector3d vel;
  for (int i = 0; i < 3; ++i) {
    double x, v;
    planners_[i].getPosVel(t, &x, &v);
    vel(i) = v;
  }
  return vel;
}

double LinearPlanner3dPath::duration() const {
  return max(planners_[0].duration(), max(planners_[1].duration(), planners_[2].duration()));
}

Vector3d LinearPlanner3dPath::initialAccelerationDirection() const {
  // TODO is this a good way of doing it?
  return velocity(1e-4).normalized();
}
