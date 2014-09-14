#ifndef MEMSIMU_H
#define MEMSIMU_H

#include <mutex>
#include <stdint.h>
using namespace std;

#include <Eigen/Geometry>
using namespace Eigen;

#include "quadstate.h"

class Sensors;

class SavedState {
public:
  int64_t time;
  QuadState state;
};

/**
 * IMU integrates input from the MEMS sensors together with position and orientation estimation
 * from a camera to obtain an estimate of the quad state. The camera is used to periodically
 * update the position and orientation to a more accurate value without drift, and, by maintaining
 * two saved states, we can also update the estimate on the velocity which is also subject to
 * drift.
 */
class Imu {
public:
  Imu(int environment);

  /**
   * Returns the last updated-to state.
   */
  QuadState lastState() const;

  /**
   * Return a best guess for the quad state at the given time, which should be
   * in the future (possibly works if in past as well, as long as not too far).
   * This function  is implemented with the expectation that time is at most
   * tens of  milliseconds into the future, since it uses the assumption that
   * the current linear and angular acceleration will not change from the last
   * point they were measured to the given time.)
   */
  QuadState stateForTime(int64_t time) const;

  /**
   * The last sensor acceleration reading (possibly corrected for bias - not yet
   * implemented at the time of writing this comment).
   */
  Vector3d lastAcceleration() const;
  /**
   * Similar as above, but for angular acceleration.
   */
  Vector3d lastAngularAcceleration() const;

  void setUpdating(bool updating) { updating_ = updating; }
  void reset();

private:
  void run();
  void cameraRun();
  QuadState step(QuadState const& initial, Vector3d const& accel_reading, Vector3d const& gyro_reading, double t) const;
  void integrate(QuadState& state, const Vector3d& accel, double t) const;

  bool updating_;
  SavedState state_;
  Vector3d last_acceleration_;
  Vector3d last_angular_acceleration_;
  Sensors *sensors_;

  /**
   * This is used when we have a camera snapshot, so that once we have established our position
   * and orientation from the camera shot (which takes some processing time), we are able to
   * calculate the difference and add this onto our current state to obtain a better estimate for
   * our current state.
   */
  SavedState state_at_camera_save_;

  mutable mutex mutex_;
  mutable QuadState k1_;
};

#endif // MEMSIMU_H
