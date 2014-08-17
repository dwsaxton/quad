#include "propellers.h"

#include <cassert>

#include "globals.h"
#include "quad.h"

Propellers::Propellers() {
  input_.setZero();
}

void Propellers::setInput(int i, double input) {
  assert(input >= 0 && input <= 1 && i >= 0 && i <= 3);
  Globals::self().simulatedQuad()->setPropInput(i, input);
  input_[i] = input;
}

void Propellers::setInput(Vector4d const& input) {
  Globals::self().simulatedQuad()->setPropInput(input);
  input_ = input;
}
