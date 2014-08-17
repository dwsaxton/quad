#include "quadraticintercept.h"

#include <iostream>
using namespace std;

#include "globals.h"
#include "linearplanner3d.h"
#include "simplequadraticintercept.h"

QuadraticIntercept::QuadraticIntercept() {
}

shared_ptr<Path> QuadraticIntercept::interceptPath(double T_hint, bool* found, LinearPlanner3d* simpleIntercept) const {
  LinearPlanner3d simple;
//   SimpleQuadraticIntercept simple;

  // Translate target
  simple.target_ = target_;
  simple.target_.c = state_.translateSpaceToBody(simple.target_.c);
  simple.target_.b = state_.rotateSpaceToBody(simple.target_.b - state_.vel);
  simple.target_.a = state_.rotateSpaceToBody(simple.target_.a + Vector3d(0, 0, 0.5 * GRAVITY));

  shared_ptr<Path> path = simple.interceptPath(T_hint, found);

  if (simpleIntercept) {
    *simpleIntercept = simple;
  }
  
  return shared_ptr<Path>(new CompletePath(path, state_));
}

CompletePath::CompletePath(shared_ptr< Path > original, const QuadState& quad_state) {
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

