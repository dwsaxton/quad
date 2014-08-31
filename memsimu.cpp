#include "memsimu.h"

#include <iostream>
#include <thread>
using namespace std;

#include "globals.h"
#include "sensors.h"
#include "quad.h"

const int refresh_us = 10000; // 10 ms

MemsImu::MemsImu(int environment) {
  sensors_ = new Sensors(environment);
  last_acceleration_.setZero();
  last_angular_acceleration_.setZero();
  new thread(&MemsImu::run, this);
}

void MemsImu::run() {
  Globals::self().sleep_us(100000); // sleep 0.1 seconds to allow stuff to come online
  int64_t start_time = Globals::self().currentTime_us();
  time_ = start_time;
  int64_t iteration = 0;
  while (true) {
    iteration++;
    mutex_.lock();
    last_acceleration_ = sensors_->readAccelerometer();
    last_angular_acceleration_ = sensors_->readGyroscope();
//     cout << "accel: " << last_acceleration_.transpose() << endl;
//     cout << "gyro: " << last_angular_acceleration_.transpose() << endl;
    int64_t new_time = Globals::self().currentTime_us();
    state_ = step(state_, last_acceleration_, last_angular_acceleration_, (new_time - time_) * 1e-6);
    time_ = new_time;
    mutex_.unlock();
    Globals::self().sleepUntil_us(start_time + iteration * refresh_us);
  }
}

QuadState MemsImu::stateForTime(int64_t time) const {
  lock_guard<mutex> lock(mutex_);
  // TODO this!!!
  return step(Globals::self().simulatedQuad()->state(), last_acceleration_, last_angular_acceleration_, (time - time_) * 1e-6);
//   return step(state_, last_acceleration_, last_angular_acceleration_, (time - time_) * 1e-6);
}

QuadState MemsImu::step(QuadState const& initial, Vector3d const& accel_reading, Vector3d const& gyro_reading, double t) const {
  QuadState state(initial);
  state.omega = gyro_reading;
  integrate(state, accel_reading - GRAVITY * state.rotateBodyToSpace(ez), t);
  return state;
}

void MemsImu::integrate(QuadState& state, const Vector3d& accel, double t) const {
  state.calcDeriv(&k1_, accel);
  state += k1_ * t;
  state.orient.normalize();
}

void MemsImu::reset() {
  lock_guard<mutex> lock(mutex_);
  state_.reset();
}

