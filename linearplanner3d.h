#ifndef LINEARPLANNER3D_H
#define LINEARPLANNER3D_H

#include <memory>

#include "linearplanner.h"
#include "path.h"
#include "pathinterceptplanner.h"

void TestLinearPlanner3d();

class LinearPlanner3dPath;

class LinearPlanner3d : public PathInterceptPlanner {
public:
  shared_ptr<Path> interceptPath(double T_hint, bool* found) const;

  double f(double T, bool useMono) const;

private:
  double durationCost(LinearPlanner3dPath const& path, bool useMono) const;
  LinearPlanner3dPath calcInterceptPathAtT(double T, bool useMono) const;
  LinearPlanner3dPath interceptPath(bool useMono, bool* found) const;

  friend void TestLinearPlanner3d();
};

class LinearPlanner3dPath : public Path {
public:
  LinearPlanner3dPath(bool mono_in_z);

  /**
   * Initializes the planners for the given target position and velocity.
   * @return the duration taken to get to here.
   */
  double initForTarget(Vector3d const& pos, Vector3d const& vel);
  /**
   * Adjust each of the coordinate planners for the given duration
   */
  void adjustForDuration(double duration);

  bool isValid(double *penalty = 0) const;
  Vector3d position(double t) const;
  Vector3d velocity(double t) const;
  double duration() const;
  Vector3d initialAccelerationDirection() const;
  std::shared_ptr<Planner1d> planners_[3];
};

#endif // LINEARPLANNER3D_H
