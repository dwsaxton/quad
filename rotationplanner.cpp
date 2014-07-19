#include "rotationplanner.h"

#include <cassert>
#include <cmath>
#include <Eigen/Core>
#include <iostream>
using namespace std;

#include "linearplanner.h"
#include "quadstate.h"

RotationPlanner::RotationPlanner() {
  max_pitch_acceleration_ = -1;
}

void RotationPlanner::calcNextStep(double time_step, Vector3d *heading, Vector3d *omega) const {
  Vector3d a = current_heading_;
  Vector3d b = target_heading_;
  Vector3d v = current_omega_.cross(a); // current velocity

  // calculate vector for great circle direction
  Vector3d d = b - a;
  d -= a.dot(d) * a;
  if (d.norm() < 0.02) {
    // TODO is this a good value? this strategy is a bit arbitrary.
    d = v; // use velocity to define the great circle. (any great circle will roughly go through the point)
  }
  d.normalize();

  // And transverse direction
  Vector3d e = a.cross(d);

  // Split v into components that are parallel and transverse to the great circle
  double vx = d.dot(v);
  double vy = e.dot(v);

  // TODO possibly want to "spin all the way around" more than once, and find the best target

  // angular distance to travel
  double distance = acos(a.dot(b));

  // "x" component
  LinearPlanner planner;
  planner.max_accel = max_pitch_acceleration_;
  planner.time_step = time_step;
  planner.v0 = vx;
  planner.v1 = 0;
  planner.x0 = 0;
  planner.x1 = distance;
  double x1, v1;
  planner.calcNext(&x1, &v1);
  
  // "y" component
  planner.v0 = vy;
  planner.v1 = 0;
  planner.x0 = 0;
  planner.x1 = 0;
  double x2, v2;
  planner.calcNext(&x2, &v2);

  // new heading
  Vector3d c = a + x1 * d + x2 * e;
  c.normalize(); // TODO this "project back to circle" isn't great if we're taking a big jump..
  *heading = c;

  // new velocity of heading
  Vector3d w = v1 * d + v2 * e; // TODO again not great if out of linear range

  double current_spin = current_omega_.dot(current_heading_);
  double new_spin_abs = max(0.0, abs(current_spin) - time_step*max_pitch_acceleration_); // TODO also max physical pitch acceleration about z axis is likely to be different
  double new_spin = current_spin > 0 ? new_spin_abs : -new_spin_abs;

  // General solution for angular velocity is of the form omega = c \cross v + \alpha * c, for any \alpha.
  // We want the z-axis spin to be new_spin, so this gives \alpha = new_spin
  *omega = c.cross(w) + new_spin * c;
}
