#include "imu.h"

#include <iostream>
#include <thread>
using namespace std;

#include "globals.h"
#include "sensors.h"
#include "quad.h"

const int refresh_us = 10000; // 10 ms

Imu::Imu(int environment) {
  updating_ = true;
  sensors_ = new Sensors(environment);
  last_acceleration_.setZero();
  last_angular_acceleration_.setZero();
  new thread(&Imu::run, this);
  new thread(&Imu::cameraRun, this);
}

void Imu::run() {
  Globals::self().sleep_us(100000); // sleep 0.1 seconds to allow stuff to come online
  int64_t start_time = Globals::self().currentTime_us();
  state_.time = start_time;
  int64_t iteration = 0;
  while (true) {
    iteration++;
    mutex_.lock();
    last_acceleration_ = sensors_->readAccelerometer();
    last_angular_acceleration_ = sensors_->readGyroscope();
    int64_t new_time = Globals::self().currentTime_us();
    if (updating_) {
      double time_step = (new_time - state_.time) * 1e-6;
//       cout << "ts = " << time_step << endl;
      state_.state = step(state_.state, last_acceleration_, last_angular_acceleration_, time_step);
    }
    state_.time = new_time;
    mutex_.unlock();
    Globals::self().sleepUntil_us(start_time + iteration * refresh_us);
  }
}

// TODO can we make this thread lower priority?
void Imu::cameraRun() {
  Globals::self().sleep_us(100000); // sleep 0.1 seconds to allow stuff to come online
  while (true) {
    int64_t prev_camera_state_time = state_at_camera_save_.time;

    mutex_.lock();
    // TODO get camera image. Also do we need to lock the mutex while getting the camera image? How long does it take?
    state_at_camera_save_ = state_;
    mutex_.unlock();

    // NOTE this simulates doing image processing on the camera image
    Vector3d pos = Globals::self().simulatedQuad()->state().pos;
    Quaterniond orient = Globals::self().simulatedQuad()->state().orient;
    Globals::self().sleep_us(100000);

    Vector3d pos_error = state_at_camera_save_.state.pos - pos;
    double dt = (state_at_camera_save_.time - prev_camera_state_time) * 1e-6;
    Vector3d vel_error = pos_error / dt;

    mutex_.lock();
    state_.state.pos -= pos_error;
    state_.state.vel -= vel_error;
    state_.state.orient = orient * state_at_camera_save_.state.orient.inverse() * state_.state.orient;
    mutex_.unlock();
  }
}

QuadState Imu::stateForTime(int64_t time) const {
  lock_guard<mutex> lock(mutex_);
  return step(state_.state, last_acceleration_, last_angular_acceleration_, (time - state_.time) * 1e-6);
}

QuadState Imu::step(QuadState const& initial, Vector3d const& accel_reading, Vector3d const& gyro_reading, double t) const {
  QuadState state(initial);
  state.omega = gyro_reading;
  integrate(state, accel_reading - GRAVITY * state.rotateBodyToSpace(ez), t);
  return state;
}

void Imu::integrate(QuadState& state, const Vector3d& accel, double t) const {
  state.calcDeriv(&k1_, accel);
  state += k1_ * t;
  state.orient.normalize();
}

void Imu::reset() {
  lock_guard<mutex> lock(mutex_);
  state_.state.reset();
}

QuadState Imu::lastState() const {
  lock_guard<mutex> lock(mutex_);
  return state_.state;
}

Vector3d Imu::lastAcceleration() const {
  lock_guard<mutex> lock(mutex_);
  return last_acceleration_;
}

Vector3d Imu::lastAngularAcceleration() const {
  lock_guard<mutex> lock(mutex_);
  return last_angular_acceleration_;
}
