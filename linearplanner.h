#ifndef LINEARPLANNER_H
#define LINEARPLANNER_H
#include <time.h>

/**
 * We have an initial position and velocity, and wish to achieve a target position and velocity.
 */
class LinearPlanner {
public:
  LinearPlanner();

  void setInitial(double x, double v);
  double setTarget(double x, double v);
  double setMaxAccel(double max_accel);

  void getPosVel(double time, double *x, double *v) const;
  double duration() const;

private:
  void plan() const;

  double x0;
  double v0;
  double x1;
  double v1;
  double max_accel_;

  mutable double t0;
  mutable double t1;
  mutable double a;
  mutable double params_changed_;
};

#endif // LINEARQUADRATICPLANNER_H
