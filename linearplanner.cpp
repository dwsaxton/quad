#include "linearquadraticplanner.h"
#include "linearplanner.h"

LinearPlanner::LinearPlanner() {
  x0 = 0;
  v0 = 0;
  x1 = 0;
  v1 = 0;
  time_step = 0;
  max_accel = 0;
}

bool shouldAccelRight(double a, double y, double w) {
  return w > 0 || w*w < a*y;
}

double accelRightTime(double a, double y, double w) {
  assert(shouldAccelRight(a, y, w));
  return (2*w - sqrt(6*max_accel*y - 2*w*w)) / (3 * max_accel);
}

void LinearPlanner::calcNext(double* x, double* v) const {
  calcNext(shouldAccelRight(max_accel, y, w) ? RightThenLeft : LeftThenRight, x, v);
}

void LinearPlanner::calcNext(LinearPlanner::Hint hint, double* x, double* v) const {
  double y = x1 - x0;
  double w = v1 - v0;
  bool flip = y < 0;
  if (flip) {
    y *= -1;
    w *= -1;
  }

  switch (hint) {
    case NoAccel:
      break; // constant speed
    case JustRight:
    case RightThenLeft: {
      double accel_right_time = accelRightTime(max_accel, y, w);
      if (accel_right_time >= time_step) {
        // Great, don't have to do any further calculations!
        *x = 0.5 * max_accel * time_step * time_step;
        *v = max_accel * time_step;
        break;
      }
      
      // Accelerate right for a little bit, then left
      LinearPlanner planner(*this);
      planner.x0 += 0.5 * max_accel * accel_right_time * accel_right_time;
      planner.v0 += max_accel * accel_right_time;
      planner.time_step -= accel_right_time;
      planner.calcNext(hint == RightThenLeft ? JustLeft : NoAccel, x, v);
      break;
    }
    case JustLeft:
    case LeftThenRight: {
      double accel_left_time = accelLeftTime(max_accel, y, w);
      if (accel_left_time >= time_step) {
        // Great, don't have to do any further calculations!
        *x = - 0.5 * max_accel * time_step * time_step;
        *v = - max_accel * time_step;
        break;
      }
      
      // Accelerate right for a little bit, then left
      LinearPlanner planner(*this);
      planner.x0 -= 0.5 * max_accel * accel_left_time * accel_left_time;
      planner.v0 -= max_accel * accel_left_time;
      planner.time_step -= accel_left_time;
      planner.calcNext(hint == LeftThenRight ? JustRight : NoAccel, x, v);
      break;
    }
  }

  // "un-normalize" the final result
  if (flip) {
    *x *= -1;
    *w *= -1;
  }
  *x += x0 + v0 * time_step;
  *w += v0;
}
