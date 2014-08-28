#ifndef MEMSIMU_H
#define MEMSIMU_H

#include <mutex>
#include <stdint.h>
using namespace std;

#include <Eigen/Geometry>
using namespace Eigen;

#include "quadstate.h"

class Sensors;

class MemsImu {
public:
  MemsImu(int environment);

  // For the following three functions, see the corresponding descriptions in
  // the Imu class.
  QuadState stateForTime(int64_t time) const;
  Vector3d lastAcceleration() const { return last_acceleration_; }
  Vector3d lastAngularAcceleration() const { return last_angular_acceleration_; }
  void reset();

private:
  void run();
  QuadState step(QuadState const& initial, Vector3d const& accel_reading, Vector3d const& gyro_reading, double t) const;
  void integrate(QuadState& state, const Vector3d& accel, double t) const;

  int64_t time_;
  QuadState state_;
  Vector3d last_acceleration_;
  Vector3d last_angular_acceleration_;
  Sensors *sensors_;

  mutable mutex mutex_;
  mutable QuadState k1_; // TODO have to ensure this isn't used across multiple threads?
};

#endif // MEMSIMU_H
