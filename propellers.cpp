#include "propellers.h"

#include <cassert>

#include "globals.h"
#include "hw/pca9685.h"
#include "quad.h"

Propellers::Propellers(int environment) {
  input_.setZero();
  if (environment == Globals::OnBoard) {
    pwm_ = new Pca9685();
  } else {
    pwm_ = nullptr;
  }
}

void Propellers::setInput(int i, double input) {
  assert(input >= 0 && input <= 1 && i >= 0 && i <= 3);
  if (pwm_ != nullptr) {
    pwm_->setPWM(i, 0.1 + 0.1 * input);
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
