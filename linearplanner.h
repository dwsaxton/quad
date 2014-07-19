#ifndef LINEARPLANNER_H
#define LINEARPLANNER_H
#include <time.h>

/**
 * We have an initial position and velocity, and wish to achieve a target position and velocity.
 */
class LinearPlanner {
public:
  LinearPlanner();

  double x0;
  double v0;
  double x1;
  double v1;
  double max_accel;
  double time_step;

  void calcNext(double *x, double *v) const;
};

#endif // LINEARQUADRATICPLANNER_H
