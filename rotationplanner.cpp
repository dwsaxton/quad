#include "rotationplanner.h"

#include <cassert>
#include <cmath>
#include <Eigen/Core>
#include <iostream>
using namespace std;

#include "quadstate.h"

// taken from http://stackoverflow.com/questions/1031005/is-there-an-algorithm-for-converting-quaternion-rotations-to-euler-angle-rotatio
// In a right-handed Cartesian coordinate system with Z axis pointing up:
void GetEulerAngles(Quaterniond q, double& yaw, double& pitch, double& roll)
{
  const double w2 = q.w()*q.w();
  const double x2 = q.x()*q.x();
  const double y2 = q.y()*q.y();
  const double z2 = q.z()*q.z();
  const double unitLength = w2 + x2 + y2 + z2;    // Normalised == 1, otherwise correction divisor.
  const double abcd = q.w()*q.x() + q.y()*q.z();
  const double eps = 1e-7;    // TODO: pick from your math lib instead of hardcoding.
  if (abcd > (0.5-eps)*unitLength)
  {
    yaw = 2 * atan2(q.y(), q.w());
    pitch = M_PI;
    roll = 0;
  }
  else if (abcd < (-0.5+eps)*unitLength)
  {
    yaw = -2 * ::atan2(q.y(), q.w());
    pitch = -M_PI;
    roll = 0;
  }
  else
  {
    const double adbc = q.w()*q.z() - q.x()*q.y();
    const double acbd = q.w()*q.y() - q.x()*q.z();
    yaw = ::atan2(2*adbc, 1 - 2*(z2+x2));
    pitch = ::asin(2*abcd/unitLength);
    roll = ::atan2(2*acbd, 1 - 2*(y2+x2));
  }
}

RotationPlanner::RotationPlanner() {
  max_pitch_acceleration_ = -1;
}

double next(double distance, double vel, double max_accel) {
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

  // angular distance to travel
  double distance = acos(a.dot(b));
  
  // TODO finish using LinearPlanner
}

#if 0
void RotationPlanner::calcNextStep(double time_step, Quaterniond *orientation, Vector3d *omega) const {
  // If the current angular velocity is so great that we couldn't decelerate to a stop in less
  // than one half circle, then use the logic that we just try to stop ourselves spinning by
  // applying force in the opposite direction of rotation
  Vector3d yaw_free_current_omega = current_omega_;
//   yaw_free_current_omega.z() = 0;
  double angular_speed = yaw_free_current_omega.norm();
  double distance_to_stop = angular_speed * angular_speed / 2 / max_pitch_acceleration_;
  double reduced_angular_speed = std::max(0.0, angular_speed - time_step * max_pitch_acceleration_);
  if (distance_to_stop > M_PI) {
    cout << "RotationPlanner::calcNextStep: applying breaks!" << endl;
    *omega = yaw_free_current_omega * reduced_angular_speed / angular_speed;
//     omega->z() = current_omega_.z();
    *orientation = current_orientation_ + time_step * derivative(current_orientation_, *omega);
    orientation->normalize();
    return;
  }

  // cheap way of getting next stop: evolve the orientation to where it would have been at the
  // current angular velocity, and then slerp it a little bit towards the target.
  // TODO this is not the best way of doing it! we're going to get oscillation, or at least we're
  // not going to converge in the optimal way possible

  double time_to_rotate = 10 * sqrt(distance_to_stop / max_pitch_acceleration_); // constant factor at start is to allow some lee-way
  *orientation = current_orientation_ + time_step * derivative(current_orientation_, current_omega_);
  double prop = std::min(1.0, time_step / time_to_rotate);
//   cout << "prop = " << prop << endl;
  *orientation = orientation->slerp(prop, target_orientation_);

  Quaterniond deriv = (1.0 / time_step) * (*orientation + (-1) * current_orientation_);
  *omega = (-2 * deriv * current_orientation_.conjugate()).vec();
//   cout << "omega: " << omega->transpose() << endl;
}
#endif
