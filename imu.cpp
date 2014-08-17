#include "imu.h"

#include "memsimu.h"

Imu::Imu() {
  mems_imu_ = new MemsImu();
}

Imu::~Imu() {
  delete mems_imu_;
}

QuadState Imu::stateForTime(int64_t time) const {
  return mems_imu_->stateForTime(time);
}

Vector3d Imu::lastAcceleration() const {
  return mems_imu_->lastAcceleration();
}

Vector3d Imu::lastAngularAcceleration() const {
  return mems_imu_->lastAngularAcceleration();
}

void Imu::reset() {
  mems_imu_->reset();
}
