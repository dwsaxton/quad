#ifndef PATHINTERCEPTPLANNER_H
#define PATHINTERCEPTPLANNER_H

#include <memory>
using namespace std;

#include "quadratic3d.h"

class Path;

void TestPathInterceptPlanners();

// TODO put this in a separate math util class?
double newtonSearch(std::function<double (double)> fn, double x, bool *found);
double binarySearch(std::function<double (double)> fn, double left, double right, bool *found);

/**
 * Calculate an intercept to a path, starting from the origin pointing upwards.
 */
class PathInterceptPlanner {
public:
  PathInterceptPlanner();

  // target trajectory, where T = 0 is the current time
  Quadratic3d target_;

  // Calculate the intercept path for the given target, under the various acceleration constraints.
  // The pointer found is set to true or false depending on whether a valid path was found. If no
  // trajectory was found, then will return the trajectory to the current target_ position
  // (at t=0). The accel_duration is how long we should accelerate for max_linear_acceleration_.
  // The length is how long the intercept path is.
  virtual shared_ptr<Path> interceptPath(double T_hint, bool* found) const = 0;
};

#endif // PATHINTERCEPTPLANNER_H
