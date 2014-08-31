#include "propellers.h"

#include <cassert>
#include <iostream>
using namespace std;

#include "globals.h"
#include "hw/pca9685.h"
#include "quad.h"

const float Propellers_PWM_min = 0.1;
const float Propellers_PWM_range = 0.1;

Propellers::Propellers(int environment) {
  input_.setZero();
  if (environment == Globals::OnBoard) {
    pwm_ = new Pca9685();
    pwm_->setPWMFreq(100);
    calibrate();
  } else {
    pwm_ = nullptr;
  }
}

void Propellers::setInput(int i, double input) {
  assert(input >= 0 && input <= 1 && i >= 0 && i <= 3);
  if (pwm_ != nullptr) {
    float speed = sqrt(input); // translate desired thrust to a speed, since thrust proportional to speed squared
    pwm_->setPWM(i, Propellers_PWM_min + Propellers_PWM_range * speed);
    if (i == 0) {
      cout << "setting to speed="<<speed<<endl;
    }
  } else {
    Globals::self().simulatedQuad()->setPropInput(i, input);
  }
  input_[i] = input;
}

void Propellers::setInput(Vector4d const& input) {
  for (int i = 0; i < 4; ++i) {
    setInput(i, input[i]);
  }
}

void Propellers::calibrate() {
  if (pwm_ == nullptr) {
    cout << "Propellers: nothing to calculate since we aren't doing PWM" << endl;
    return;
  }

  cout << "Calibrating ESCs..." << endl;
  for (int i=0; i < 4; ++i) {
    pwm_->setPWM(i, Propellers_PWM_min + Propellers_PWM_range);
  }
  cout << "Press enter after beeps" << endl;
  cin.get();
  cout << "Turning down PWM to min..." << endl;
  for (int i=0; i < 4; ++i) {
    pwm_->setPWM(i, Propellers_PWM_min);
  }
  cout << "Should be calibrated, press any button to continue!" << endl;
  cin.get();
}
