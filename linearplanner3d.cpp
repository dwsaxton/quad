#include "linearplanner3d.h"

#include <iostream>
using namespace std;

#include "linearplannermono.h"
#include "world.h"

shared_ptr<Path> LinearPlanner3d::interceptPath(double T_hint, bool *found) const {
  bool foundMono;
  LinearPlanner3dPath monoPath = interceptPath(true, &foundMono);
  bool foundFull;
  LinearPlanner3dPath fullPath = interceptPath(false, &foundFull);

  if (!foundMono || !monoPath.isValid()) {
    // Try to find it again using binary search technique
    double left = TsControllerTarget();
    double right = 1e4;
    for (int i = 0; i < 20; ++i) {
      double mid = (left + right) / 2;
      if (calcInterceptPathAtT(mid, true).isValid()) {
        right = mid;
      } else {
        left = mid;
      }
    }
    monoPath = calcInterceptPathAtT(right, true);
    monoPath.adjustForDuration(right);
    foundMono = true;
  }
  
  bool useMono = foundMono && monoPath.isValid() && durationCost(monoPath, true) < durationCost(fullPath, false);
  LinearPlanner3dPath path = useMono ? monoPath : fullPath;
  *found = useMono ? true : foundFull;
  double T = path.duration();

  cout << "usingMono: " << useMono << " T: " << T <<" path.position(T)=" << path.position(T).transpose() << " target: " << target_.eval(T).transpose()
      << " path.velocity(T)=" << path.velocity(T).transpose() << " target vel: " << target_.derivative().eval(T).transpose() << endl;
  return shared_ptr<Path>(new LinearPlanner3dPath(path));
}

LinearPlanner3dPath LinearPlanner3d::interceptPath(bool useMono, bool* found) const {
  // Do search for correct T
  double left = 1e-4; // always search for positive time. (can't fly backwards in time!)
  double right = 10;
  double valueLeft = f(left, useMono);
  while ((f(right, useMono) > 0) == (valueLeft > 0) && right < 1e5) {
    right *= 2;
  }
  if (right >= 1e6) {
    *found = false;
    return calcInterceptPathAtT(0, useMono);
  }

  auto function = [&] (double x) { return f(x, useMono); };

  // TODO passing of eps, iteration parameters
  double T = binarySearch(function, 0.0001, 100, found);
  if (T < 0.1) {
    T = 0.1;
  }
//   T = newtonSearch(function, T, found);
  return calcInterceptPathAtT(T, useMono);
}

LinearPlanner3dPath LinearPlanner3d::calcInterceptPathAtT(double T, bool useMono) const {
  Vector3d pos = target_.eval(T);
  Vector3d vel = target_.derivative().eval(T);
  LinearPlanner3dPath path(useMono);
  path.initForTarget(pos, vel);
  return path;
}

double LinearPlanner3d::f(double T, bool useMono) const {
  LinearPlanner3dPath path = calcInterceptPathAtT(T, useMono);
  double path_duration = path.duration();
  // Translate an invalid path into a large time penalty so we avoid them.
  double time_penalty = 0;
  path.isValid(&time_penalty);
  return path.duration() + 100 * time_penalty - T;
}

double LinearPlanner3d::durationCost(LinearPlanner3dPath const& path, bool useMono) const {
  double d = path.duration();
  
  Vector3d initial_direction = path.initialAccelerationDirection();
  double angle = acos(initial_direction.z());
  double angle_threshold = M_PI / 4; // TODO should this be standardized somewhere?
  double rotation_required = max(0.0, angle - angle_threshold);

  // TODO this is not a good estimate for the length of time required for rotation
  d += 4 * sqrt(rotation_required / MaxPitchAcceleration);

  if (!useMono) {
    d += 8; // penalty for not using mono-acceleration
  }
  return d;
}


LinearPlanner3dPath::LinearPlanner3dPath(bool mono_in_z) {
  planners_[0] = shared_ptr<Planner1d>(new LinearPlanner());
  planners_[1] = shared_ptr<Planner1d>(new LinearPlanner());
  planners_[2] = shared_ptr<Planner1d>(
      mono_in_z ? static_cast<Planner1d*>(new LinearPlannerMono()) : static_cast<Planner1d*>(new LinearPlanner()));
}

double LinearPlanner3dPath::initForTarget(const Vector3d& pos, const Vector3d& vel) {
  double duration = 0;
  for (int i = 0; i < 3; ++i) {
    planners_[i]->setTarget(pos[i], vel[i]);
    duration = max(duration, planners_[i]->getMinDuration(MaxLinearAcceleration / (i == 2 ? 1 : 2)));
  }
  adjustForDuration(duration);
  return duration;
}

void LinearPlanner3dPath::adjustForDuration(double duration) {
  for (int i = 0; i < 3; ++i ) {
    planners_[i]->setupForDuration(duration);
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
  double t = TsControllerTarget();
//   return (velocity(t) + 100*Vector3d(0, 0, 0.5*t*t*MaxLinearAcceleration)).normalized();
  return velocity(7*t).normalized();
}

bool LinearPlanner3dPath::isValid(double* penalty) const {
  bool valid = true;
  if (penalty) {
    *penalty = 0;
  }
  for (int i = 0; i < 3; ++i) {
    double local_penalty = 0;
    if (!planners_[i] || !planners_[i]->isValid(&local_penalty)) {
      valid = false;
      if (penalty) {
        *penalty += local_penalty;
      }
    }
  }
  return valid;
}

void TestLinearPlanner3d() {
  LinearPlanner3d planner;
  planner.target_ = Quadratic3d({0, -3, 4}, {0, -7, -2}, {0, 2, 18});
  bool found;
  LinearPlanner3dPath path = planner.interceptPath(true, &found);
  assert(found);
  assert(path.isValid());
}
