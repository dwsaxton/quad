#include "linearplanner3d.h"

#include <iostream>
using namespace std;

#include "linearplannermono.h"

Path* LinearPlanner3d::interceptPath(double T_hint, bool *found) const {
  bool foundMono;
  LinearPlanner3dPath monoPath = interceptPath(true, &foundMono);
  bool foundFull;
  LinearPlanner3dPath fullPath = interceptPath(false, &foundFull);
  
  bool usingMono;
  LinearPlanner3dPath path;
  if (foundMono && monoPath.isValid() && durationCost(monoPath, true) < durationCost(fullPath, false)) {
    path = monoPath;
    usingMono = true;
    *found = true;
  } else {
    path = fullPath;
    usingMono = false;
    *found = foundFull;
  }
  double T = path.duration();

  cout << "usingMono: " << usingMono << " T: " << T <<" path.position(T)=" << path.position(T).transpose() << " target: " << target_.eval(T).transpose()
      << " path.velocity(T)=" << path.velocity(T).transpose() << " target vel: " << target_.derivative().eval(T).transpose() << endl;
  return new LinearPlanner3dPath(path); // TODO massive memory leak
}

LinearPlanner3dPath LinearPlanner3d::interceptPath(bool useMono, bool* found) const {
  // Do search for correct T
  double left = 1e-4; // always search for positive time. (can't fly backwards in time!)
  double right = 10;
  double valueLeft = f(left, useMono);
  while ((f(right, useMono) > 0) == (valueLeft > 0) && right < 1e6) {
    right *= 2;
  }
  if (right >= 1e6) {
    *found = false;
    return calcInterceptPathForT(0, useMono);
  }

  auto function = [&] (double x) { return f(x, useMono); };

  // TODO passing of eps, iteration parameters
  double T = binarySearch(function, 0.0001, 100, found);
  if (T < 0.1) {
    T = 0.1;
  }
  T = newtonSearch(function, T, found);
  return calcInterceptPathForT(T, useMono);
}

LinearPlanner3dPath LinearPlanner3d::calcInterceptPathForT(double T, bool useMono) const {
  Vector3d pos = target_.eval(T);
  Vector3d vel = target_.derivative().eval(T);

  LinearPlanner3dPath path;
  if (useMono) {
    path.initMono();
  } else {
    path.initFull();
  }
  path.setTarget(max_linear_acceleration_, pos, vel);
  double duration = path.duration();
  for (int i = 0; i < 3; ++i ) {
    path.planners_[i]->setupForDuration(duration);
  }

  if (path.isValid() && (pos - path.position(T)).norm() > 0.01) {
    Vector3d position = path.position(T);
    bool bad = true;
    position = path.position(T);
  }
  return path;
}

double LinearPlanner3d::f(double T, bool useMono) const {
  LinearPlanner3dPath path = calcInterceptPathForT(T, useMono);
  return durationCost(path, useMono) - T;
}

double LinearPlanner3d::durationCost(LinearPlanner3dPath const& path, bool useMono) const {
  double d = path.duration();
  
  Vector3d initial_direction = path.initialAccelerationDirection();
  double angle = acos(initial_direction.z());
  double angle_threshold = M_PI / 4; // TODO should this be standardized somewhere?
  double rotation_required = max(0.0, angle - angle_threshold);

  // TODO this is not a good estimate for the length of time required for rotation
  d += 4 * sqrt(rotation_required / max_pitch_acceleration_);

  if (!useMono) {
    d += 4; // penalty for not using mono-acceleration
  }
  return d;
}


void LinearPlanner3dPath::initMono() {
  planners_[0] = shared_ptr<Planner1d>(new LinearPlanner());
  planners_[1] = shared_ptr<Planner1d>(new LinearPlanner());
  planners_[2] = shared_ptr<Planner1d>(new LinearPlannerMono());
}

void LinearPlanner3dPath::initFull() {
  for (int i = 0; i < 3; ++i) {
    planners_[i] = shared_ptr<Planner1d>(new LinearPlanner());
  }
}

void LinearPlanner3dPath::setTarget(double max_accel, Vector3d const& pos, Vector3d const& vel) {
  for (int i = 0; i < 3; ++i) {
    planners_[i]->setTarget(pos[i], vel[i]);
    planners_[i]->setupForMaxAccel(max_accel / (i == 2 ? 1 : 4) /* TODO this is smudge hack factor */ );
  }
}

Vector3d LinearPlanner3dPath::position(double t) const {
  Vector3d pos;
  for (int i = 0; i < 3; ++i) {
    double x, v;
    planners_[i]->getPosVel(t, &x, &v);
    pos(i) = x;
  }
  return pos;
}

Vector3d LinearPlanner3dPath::velocity(double t) const {
  Vector3d vel;
  for (int i = 0; i < 3; ++i) {
    double x, v;
    planners_[i]->getPosVel(t, &x, &v);
    vel(i) = v;
  }
  return vel;
}

double LinearPlanner3dPath::duration() const {
  return max(planners_[0]->duration(), max(planners_[1]->duration(), planners_[2]->duration()));
}

Vector3d LinearPlanner3dPath::initialAccelerationDirection() const {
  // TODO is this a good way of doing it?
  return velocity(1e-2).normalized();
}

bool LinearPlanner3dPath::isValid() const {
  for (int i = 0; i < 3; ++i) {
    if (!planners_[i] || !planners_[i]->isValid()) {
      return false;
    }
  }
  return true;
}

