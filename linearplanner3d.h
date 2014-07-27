#ifndef LINEARPLANNER3D_H
#define LINEARPLANNER3D_H

#include <memory>

#include "linearplanner.h"
#include "path.h"
#include "pathinterceptplanner.h"

class LinearPlanner3dPath;

class LinearPlanner3d : public PathInterceptPlanner {
public:
  Path *interceptPath(double T_hint, bool* found) const;

  double f(double T, bool useMono) const;

private:
  double durationCost(LinearPlanner3dPath const& path, bool useMono) const;
  LinearPlanner3dPath calcInterceptPathForT(double T, bool useMono) const;
  LinearPlanner3dPath interceptPath(bool useMono, bool* found) const;
};

class LinearPlanner3dPath : public Path {
public:
  /**
   * Create it with a mono linear planner in the Z direction.
   */
  void initMono();
  /**
   * Create it with full "linear planners" in all directions.
   */
  void initFull();

  void setTarget(double max_accel, Vector3d const& pos, Vector3d const& vel);

  bool isValid() const;
  Vector3d position(double t) const;
  Vector3d velocity(double t) const;
  double duration() const;
  Vector3d initialAccelerationDirection() const;
  std::shared_ptr<Planner1d> planners_[3];
};

#endif // LINEARPLANNER3D_H
