#include "globals.h"

#include <time.h>
#include <X11/Xlib.h>

#include "controllooper.h"
#include "imu.h"
#include "propellers.h"
#include "quad.h"
#include "hw/raspberryi2c.h"

Globals &Globals::self() {
  static Globals instance;
  return instance;
}

Globals::Globals() {
  if (XOpenDisplay(NULL)) {
    environment_ = Simulation;
  } else {
    environment_ = OnBoard;
  }

  simulated_quad_running_ = false;
  i2c_ = new RaspberryI2c();
  control_looper_ = new ControlLooper();
  imu_ = new Imu(environment_, i2c_);
  propellers_ = new Propellers();
  simulated_quad_ = new Quad();

  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  initial_seconds_ = ts.tv_sec;

  if (environment_ == Simulation) {
    new thread(&Globals::runSimulatedQuad, this);
  }
}

int64_t Globals::currentTime_us() const {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return 1000000 * (ts.tv_sec - initial_seconds_) + ts.tv_nsec / 1000;
}

void Globals::sleep_us(int64_t duration) const {
  timespec ts;
  ts.tv_sec  =  duration / 1000000;
  ts.tv_nsec = (duration % 1000000) * 1000;
  nanosleep(&ts, 0); // TODO should make use of the return value, in case error or interrupt?
};

void Globals::sleepUntil_us(int64_t time) const {
  int64_t current = currentTime_us();
  if (current >= time) {
    return;
  }
  sleep_us(time - current);
}

void Globals::reset() {
  // TODO implement
  simulated_quad_->reset();
  imu_->reset();
}

void Globals::runSimulatedQuad() {
  Globals::self().sleep_us(100000); // sleep 0.1 seconds to allow stuff to come online
  int64_t start_time = Globals::self().currentTime_us();
  int64_t iteration = 0;
  const int64_t refresh_us = TsWorldMs * 1000 / 10;
  while (true) {
    iteration++;
    if (simulated_quad_running_) {
      simulated_quad_->step(refresh_us * 1e-6);
    }
    Globals::self().sleepUntil_us(start_time + iteration * refresh_us);
  }
}
