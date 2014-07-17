#include "quadraticintercept.h"

#include <iostream>
using namespace std;

#include "simplequadraticintercept.h"
#include "world.h"

QuadraticIntercept::QuadraticIntercept() {
  max_linear_acceleration_ = -1;
  max_pitch_acceleration_ = -1;
}

Path* QuadraticIntercept::interceptPath(bool* found) const {
  assert(max_linear_acceleration_ > 0);
  assert(max_pitch_acceleration_ > 0);

  SimpleQuadraticIntercept simple;
  simple.max_linear_acceleration_ = max_linear_acceleration_;
  simple.max_pitch_acceleration_ = max_pitch_acceleration_;

  // Translate target
  simple.target_ = target_;
  simple.target_.c = state_.translateSpaceToBody(simple.target_.c);
  simple.target_.b = state_.rotateSpaceToBody(simple.target_.b - state_.vel);
  simple.target_.a = state_.rotateSpaceToBody(simple.target_.a + Vector3d(0, 0, 0.5 * 9.81));

  Path * path = simple.interceptPath(found);
//   if (info.valid) {
//     double T = info.duration;
//     Vector3d target_pos = simple.target_.eval(T);
//     Vector3d path_pos = info.quadratic.eval(1);
//     Vector3d diff = target_pos - path_pos;
//     assert(diff.norm() < 1e-3);
//   }

//   cout << "Pre-rotation length: "<< info.quadratic.length(1) << endl;

//   info.quadratic.c = state_.translateBodyToSpace(info.quadratic.c);
//   info.quadratic.b = state_.rotateBodyToSpace(info.quadratic.b);
//   info.quadratic.a = state_.rotateBodyToSpace(info.quadratic.a);

//   cout << "Post-rotation length: "<< info.quadratic.length(1) << endl;

//   cout << "Expected length: " << info.length << endl;

//   *found = info.valid;

  return new CompletePath(path, state_);

//   if (info.valid) {
//     double T = info.duration;
//     Vector3d target_pos = simple.target_.eval(T);
//     Vector3d intercept_pos = path->position(T);
//     Vector3d diff = target_pos - intercept_pos;
// //     assert(diff.norm() < 0.1); // TODO should we make it more precise?
//   }
}

CompletePath::CompletePath(Path *original, QuadState const& quad_state) {
  original_ = original;
  quad_state_ = quad_state;
}

Vector3d CompletePath::position(double t) const {
  return quad_state_.translateBodyToSpace(original_->position(t)) + quad_state_.vel * t - Vector3d(0, 0, 0.5 * 9.81 * t * t);
}

Vector3d CompletePath::velocity(double t) const {
  return quad_state_.rotateBodyToSpace(original_->velocity(t)) + quad_state_.vel - Vector3d(0, 0, 9.81 * t);
}

Vector3d CompletePath::initialAccelerationDirection() const {
  return quad_state_.rotateBodyToSpace(original_->initialAccelerationDirection());
//   // TODO this time definitely does not belong here!
//   double t = TsControllerTarget();
//   Vector3d v0 = quad_state_.rotateBodyToSpace(original_->velocity(t)) + quad_state_.vel;
//   return v0.normalized();
}

