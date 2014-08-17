#include "controllooper.h"

#include "globals.h"
#include "highercontroller.h"
#include "imu.h"
#include "propellers.h"
#include "quadstate.h"

ControlLooper::ControlLooper() {
//   m_wind.setZero();
  m_controller = new HigherController;
  is_running_ = true;
  next_input_waiting_ = false;
  new thread(&ControlLooper::run, this);
}

void ControlLooper::run() {
  Globals::self().sleep_us(100000); // sleep 0.1 seconds to allow stuff to come online

  while (true) {
    int64_t time = Globals::self().currentTime_us();
    if (is_running_) {
      step();
    }
    Globals::self().sleepUntil_us(time + TsWorldMs * 1000);
    if (next_input_waiting_) {
      Globals::self().propellers()->setInput(next_input_);
      next_input_waiting_ = false;
    }
  }
}

ControlLooper::~ControlLooper() {
   delete m_controller;
}

void ControlLooper::step() {
  Vector4d input = Globals::self().propellers()->input();
  int64_t current_time = Globals::self().currentTime_us();
  int64_t next_time = current_time + TsWorldMs * 1000;
  QuadState state = Globals::self().imu()->stateForTime(next_time);

  next_input_ = m_controller->getPropInputs(state, input);
  next_input_waiting_ = true;

//   stepWind();
}

void ControlLooper::setRunning(bool running) {
  is_running_ = running;
}

// void ControlLooper::stepWind() {
//   double maxWind = 1; // max number of newtons of force
//   
//   // Average time of change
//   double rate = 0.05 / maxWind;
//   
//   for ( int i = 0; i < 3; ++i )
//   {
//     double x = m_wind(i);
//     
//     double maxDelta = TsWorldTarget() / rate;
//     
//     x += unifRand( -maxDelta, maxDelta );
//     
//     if ( x > maxWind )
//       x = maxWind;
//     if ( x < -maxWind )
//       x = -maxWind;
//     
//     m_wind(i) = x;
//   }
//   
//   m_simulatedQuad->setWindForce(m_wind);
// }
