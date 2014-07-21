#include "quadraticintercept.h"

#include <iostream>
using namespace std;

#include "simplequadraticintercept.h"
#include "world.h"

QuadraticIntercept::QuadraticIntercept() {
  max_linear_acceleration_ = -1;
  max_pitch_acceleration_ = -1;
}

Path* QuadraticIntercept::interceptPath(double T_hint, bool* found, SimpleQuadraticIntercept* simpleIntercept) const {
  assert(max_linear_acceleration_ > 0);
  assert(max_pitch_acceleration_ > 0);

  SimpleQuadraticIntercept simple;
  simple.max_linear_acceleration_ = max_linear_acceleration_;
  simple.max_pitch_acceleration_ = max_pitch_acceleration_;

  // Translate target
  simple.target_ = target_;
  simple.target_.c = state_.translateSpaceToBody(simple.target_.c);
  simple.target_.b = state_.rotateSpaceToBody(simple.target_.b - state_.vel);
  simple.target_.a = state_.rotateSpaceToBody(simple.target_.a + Vector3d(0, 0, 0.5 * GRAVITY));

  Path * path = simple.interceptPath(T_hint, found);

  if (simpleIntercept) {
    *simpleIntercept = simple;
  }
  
  return new CompletePath(path, state_);
}

CompletePath::CompletePath(Path *original, QuadState const& quad_state) {
  original_ = original;
  quad_state_ = quad_state;
}

Vector3d CompletePath::position(double t) const {
  return quad_state_.translateBodyToSpace(original_->position(t)) + quad_state_.vel * t - Vector3d(0, 0, 0.5 * GRAVITY * t * t);
}

Vector3d CompletePath::velocity(double t) const {
  return quad_state_.rotateBodyToSpace(original_->velocity(t)) + quad_state_.vel - Vector3d(0, 0, GRAVITY * t);
}

Vector3d CompletePath::initialAccelerationDirection() const {
  return quad_state_.rotateBodyToSpace(original_->initialAccelerationDirection());
}

