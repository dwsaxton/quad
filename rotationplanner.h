#ifndef ROTATIONPLANNER_H
#define ROTATIONPLANNER_H

#include <Eigen/Geometry>
using namespace Eigen;

/**
 * Calculates a method of changing the orientation given the rotation acceleration constraints.
 */
class RotationPlanner {
public:
  RotationPlanner();

  // maximum pitch or roll acceleration (yaw acceleration will be different, but we don't care
  // about this) in radians per second per second.
  double max_pitch_acceleration_;

  // current orientation
  Quaterniond current_orientation_;

  // target orientation
  Quaterniond target_orientation_;
  
  // current instantaneous angular velocity
  Vector3d current_omega_;
  
  void calcNextStep(double time_step, Quaterniond *orientation, Vector3d *omega) const;
};

#endif // ROTATIONPLANNER_H
