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

  // current heading
  Vector3d current_heading_;

  // target heading
  Vector3d target_heading_;
  
  // current instantaneous angular velocity in the space frame
  Vector3d current_omega_;
  
  void calcNextStep(double time_step, Vector3d* heading, Vector3d* omega) const;
};

#endif // ROTATIONPLANNER_H
