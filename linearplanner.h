#ifndef LINEARPLANNER_H
#define LINEARPLANNER_H

#include "planner1d.h"

/**
 * We have an initial position and velocity, and wish to achieve a target position and velocity.
 */
class LinearPlanner : public Planner1d {
public:
  LinearPlanner();

  double getMinDuration(double max_accel) const;
  void setupForMaxAccel(double max_accel);
  void setupForDuration(double duration);
  bool isValid() const { return t0 >= 0 && t1 >= 0; }
  void getPosVel(double time, double *x, double *v) const;
  void setTarget(double x, double v);
  void setInitial(double x, double v);
  double duration() const { return t0 + t1; }

private:
  double x0;
  double v0;
  double x1;
  double v1;
  double t0;
  double t1;
  double a;
};

#endif // LINEARPLANNER_H
