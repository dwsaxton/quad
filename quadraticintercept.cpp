#include "quadraticintercept.h"

#include "simplequadraticintercept.h"

QuadraticIntercept::QuadraticIntercept() {
  max_linear_acceleration_ = -1;
  max_pitch_acceleration_ = -1;
}

Quadratic3d QuadraticIntercept::interceptPath(bool * found) const {
  assert(max_linear_acceleration_ > 0);
  assert(max_pitch_acceleration_ > 0);

  SimpleQuadraticIntercept simple;
  simple.max_linear_acceleration_ = max_linear_acceleration_;
  simple.max_pitch_acceleration_ = max_pitch_acceleration_;

  // Translate target
  simple.target_ = target_;
  simple.target_.c = state_.translateSpaceToBody(simple.target_.c);
  simple.target_.b = state_.rotateSpaceToBody(simple.target_.b - state_.vel);
  simple.target_.a = state_.rotateSpaceToBody(simple.target_.a);

  Quadratic3d intercept = simple.interceptPath(found);

  intercept.c = state_.translateBodyToSpace(intercept.c);
  intercept.b = state_.rotateBodyToSpace(intercept.b) + state_.vel;
  intercept.a = state_.rotateBodyToSpace(intercept.a);

  return intercept;
}
