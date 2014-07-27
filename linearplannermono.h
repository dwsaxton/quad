#ifndef LINEARPLANNERMONO_H
#define LINEARPLANNERMONO_H

#include "planner1d.h"

/**
 * Represents a planner for the following motion: We start at rest at the
 * origin, and wish at some time in the future to have a given position x
 * and velocity v. This is achieved by accelerating uniformly for a given
 * time and a given rate, in a positive direction. (Of course, this is not
 * always feasible.)
 */
class LinearPlannerMono : public Planner1d {
public:
  LinearPlannerMono();

  double getMinDuration(double max_accel) const;
  void setupForDuration(double duration);
  void setupForMaxAccel(double max_accel);
  bool isValid(double *penalty) const;
  void getPosVel(double time, double *x, double *v) const;
  void setTarget(double x, double v) { x1 = x; v1 = v; }
  double duration() const { return t0 + t1 + t2; }

private:
  double x1;
  double v1;
  double a; // acceleration when accelerating (from time t0 to t0 + t1)
  double t0; // hold time
  double t1; // acceleration time
  double t2; // second hold time
};

#endif // LINEARPLANNERMONO_H
