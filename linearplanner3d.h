#ifndef LINEARPLANNER3D_H
#define LINEARPLANNER3D_H

#include "linearplanner.h"
#include "path.h"
#include "pathinterceptplanner.h"

class LinearPlanner3dPath;

class LinearPlanner3d : public PathInterceptPlanner {
public:
  Path *interceptPath(double T_hint, bool* found) const;

private:
  LinearPlanner3dPath calcInterceptPathForT(double T) const;
  double f(double T) const;
};

class LinearPlanner3dPath : public Path {
public:
  Vector3d position(double t) const;
  Vector3d velocity(double t) const;
  double duration() const;
  Vector3d initialAccelerationDirection() const;

  LinearPlanner planners_[3];
};

#endif // LINEARPLANNER3D_H
