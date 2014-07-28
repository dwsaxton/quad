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
    // Use velocity to define the great circle, if it is non-zero. (any great circle will roughly go through the point)
    if (v.norm() > 0.02) {
      d = v;
    } else {
      // Just need some other vector that is perpendicular to a
      d = Vector3d(0, 0, 1).cross(a);
      if (d.norm() < 0.02) {
        d = Vector3d(0, 1, 0).cross(a);
      }
    }
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
  planner.setInitial(0, vx);
  planner.setTarget(distance, 0);
  planner.setupForMaxAccel(max_pitch_acceleration_);
  double x1, v1;
  planner.getPosVel(time_step, &x1, &v1);
  
  // "y" component
  planner.setInitial(0, vy);
  planner.setTarget(0, 0);
  planner.setupForMaxAccel(max_pitch_acceleration_);
  double x2, v2;
  planner.getPosVel(time_step, &x2, &v2);

  // new heading
  Vector3d c = a + x1 * d + x2 * e;
  c.normalize(); // TODO this "project back to circle" isn't great if we're taking a big jump..
  *heading = c;

  // new velocity of heading
  Vector3d w = v1 * d + v2 * e; // TODO again not great if out of linear range

  double current_spin = current_omega_.dot(current_heading_);
  double spin_abs = abs(current_spin);
  double new_spin_abs = spin_abs < 1e-4 ? 0 : spin_abs * max(0.0, 1 - time_step*max_pitch_acceleration_/spin_abs); // TODO also max physical pitch acceleration about z axis is likely to be different
  double new_spin = current_spin > 0 ? new_spin_abs : -new_spin_abs;

  // General solution for angular velocity is of the form omega = c \cross v + \alpha * c, for any \alpha.
  // We want the z-axis spin to be new_spin, so this gives \alpha = new_spin
  *omega = c.cross(w) + new_spin * c;
}

void TestRotationPlanner() {
  // Test when no rotation required
  RotationPlanner planner;
  Vector3d ez = {0, 0, 1};
  planner.current_heading_ = ez;
  planner.target_heading_ = ez;
  planner.current_omega_.setZero();
  Vector3d heading;
  Vector3d omega;
  planner.calcNextStep(0.1, &heading, &omega);
  assert(heading == ez);
  assert(omega.isZero());
}
