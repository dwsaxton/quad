#include "highercontroller.h"

#include <iostream>
using namespace std;

#include "controller.h"
#include "quadraticintercept.h"
#include "rotationplanner.h"
#include "world.h"

HigherController::HigherController() {
  controller_ = new Controller();
  target_pos_ = Vector3d(4, 8, 12);
  prev_intercept_duration_ = 1;
}

Vector4d HigherController::getPropInputs(Quad const *quad) {
  double time_step = TsWorldTarget(); // TODO theoretically, we shouldn't need the time step!!!
  QuadState state = quad->state();

  LinearPlanner3d sqi;
  shared_ptr<Path> intercept = interceptForTarget(state, &sqi);
//   quad->path_ = intercept; // TODO un-comment this
//   quad->intercept = sqi; // TODO And this as well

  // Sanity check
  prev_intercept_duration_ = intercept->duration();
//   cout << "Intercept duration: " << prev_intercept_duration_ << endl;
//   cout << "Pos at intercept: " << intercept->position(prev_intercept_duration_).transpose() << endl;
//   cout << "Vel at intercept: " << intercept->velocity(prev_intercept_duration_).transpose() << endl;

  Vector3d target_pos = intercept->position(time_step);
  Vector3d target_vel = intercept->velocity(time_step);
  
  // Calculate the difference in angle between our current orientation, and that required to follow
  // the required trajectory
  Vector3d current_z = state.orient.conjugate() * Vector3d(0, 0, 1);
  Vector3d target_z = intercept->initialAccelerationDirection();
//   cout << "current_z: " <<current_z.transpose()<<endl;
//   cout << "target_z:  " <<target_z.transpose()<<endl;
//   cout << "current pos: " << state.pos.transpose() << endl;
//   cout << "target pos:  " << target_pos.transpose() << endl;
//   cout << "current vel: " << state.vel.transpose() << endl;
//   cout << "target vel:  " << target_vel.transpose() << endl;

  QVector<ControlledOutput> outputs(QUAD_STATE_SIZE);

  RotationPlanner rotation_planner;
  rotation_planner.current_heading_ = current_z;
  rotation_planner.current_omega_ = state.rotateBodyToSpace(state.omega);
  rotation_planner.max_pitch_acceleration_ = MaxPitchAcceleration;
  rotation_planner.target_heading_ = target_z;

  Vector3d next_heading;
  Vector3d next_omega;
  rotation_planner.calcNextStep(time_step, &next_heading, &next_omega);
  next_omega = state.rotateSpaceToBody(next_omega);
  
  Quaterniond next_orient; // TODO implement this
  for (int j = 0; j < 4; ++j) {
    outputs[QuadState::StateIndexOrient + j].weight = 0;
//     outputs[QuadState::StateIndexOrient + j].value[0] = next_orient.coeffs()[j];
  }
  for (int j = 0; j < 3; ++j) {
    outputs[QuadState::StateIndexomega + j].weight = 20;
    outputs[QuadState::StateIndexomega + j].value = next_omega.coeff(j);
  }

  for (int j = 0; j < 3; ++j) {
    outputs[QuadState::StateIndexPos + j].weight = 10;
    outputs[QuadState::StateIndexPos + j].value = target_pos.coeff(j);
    outputs[QuadState::StateIndexVel + j].weight = 10;
    outputs[QuadState::StateIndexVel + j].value = target_vel.coeff(j);
  }

  return controller_->getPropInputs(quad, outputs);
}

shared_ptr<Path> HigherController::interceptForTarget(QuadState const& state, LinearPlanner3d *simpleIntercept) const {
  QuadraticIntercept intercept;
  intercept.state_ = state;
  intercept.target_ = stateTargetToQuadratic();
  bool found;
  shared_ptr<Path> path = intercept.interceptPath(prev_intercept_duration_ - TsWorldTarget(), &found, simpleIntercept);
  if (!found) {
    cout << "Warning: intercept path not found!" << endl;
  }
  return path;
}

Quadratic3d HigherController::stateTargetToQuadratic() const {
  Quadratic3d quadratic;
  quadratic.c = target_pos_;
  return quadratic;
}
