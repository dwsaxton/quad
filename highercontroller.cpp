#include "highercontroller.h"

#include <iostream>
using namespace std;

#include "controller.h"
#include "quadraticintercept.h"
#include "rotationplanner.h"
#include "world.h"

// TODO these constants may be hard coded elsewhere (so check!), and besides, we should calculate them
const double MaxLinearAcceleration = 20;
const double MaxPitchAcceleration = 5;

HigherController::HigherController() {
  have_control_ = false;
  controller_ = new Controller();
  target_pos_ = Vector3d(4, 4, 12);
}

void HigherController::giveControl(bool haveControl) {
  have_control_ = haveControl;
}

void HigherController::step() {
  if (!have_control_) {
    return;
  }

  double time_step = TsControllerTarget();
  QuadState state = World::self()->simulatedQuad()->state();

  // TODO this is massive memory leak
  Path *intercept = interceptForTarget(state);
  World::self()->simulatedQuad()->path_ = intercept;

  // Sanity check
  double intercept_duration = intercept->duration();
//   cout << "Intercept duration: " << intercept_duration << endl;
//   cout << "Pos at intercept: " << intercept->position(intercept_duration).transpose() << endl;
//   cout << "Vel at intercept: " << intercept->velocity(intercept_duration).transpose() << endl;

  Vector3d target_pos = intercept->position(time_step);
  Vector3d target_vel = intercept->velocity(time_step);
  
  // Calculate the difference in angle between our current orientation, and that required to follow
  // the required trajectory
  Vector3d current_z = state.orient.conjugate() * Vector3d(0, 0, 1);
  Vector3d target_z = intercept->initialAccelerationDirection();
//   cout << "current_z: " <<current_z.transpose()<<endl;
//   cout << "target_z:  " <<target_z.transpose()<<endl;
  cout << "current pos: " << state.pos.transpose() << endl;
  cout << "target pos:  " << target_pos.transpose() << endl;
  cout << "current vel: " << state.vel.transpose() << endl;
  cout << "target vel:  " << target_vel.transpose() << endl;

  QVector<ControlledOutput> outputs(QUAD_STATE_SIZE);

  RotationPlanner rotation_planner;
  rotation_planner.current_heading_ = current_z;
  rotation_planner.current_omega_ = state.omega;
  rotation_planner.max_pitch_acceleration_ = MaxPitchAcceleration;
  rotation_planner.target_heading_ = target_z;

  Quaterniond next_orient;
  Vector3d next_omega;
  rotation_planner.calcNextStep(time_step, &next_orient, &next_omega);
  for (int j = 0; j < 4; ++j) {
    outputs[QuadState::StateIndexOrient + j].used = false;
    outputs[QuadState::StateIndexOrient + j].weight = 10;
    outputs[QuadState::StateIndexOrient + j].value[0] = next_orient.coeffs()[j];
  }
  for (int j = 0; j < 3; ++j) {
    outputs[QuadState::StateIndexomega + j].used = true;
    outputs[QuadState::StateIndexomega + j].weight = 20;
    outputs[QuadState::StateIndexomega + j].value[0] = next_omega.coeff(j);
  }

//   if (angle < M_PI/4 /* TODO used elsewhere too, shouldn't be hard-coded */) {
//   if (false) {
    for (int j = 0; j < 3; ++j) {
      outputs[QuadState::StateIndexPos + j].used = true;
      outputs[QuadState::StateIndexPos + j].weight = 10;
      outputs[QuadState::StateIndexPos + j].value[0] = target_pos.coeff(j);
      outputs[QuadState::StateIndexVel + j].used = true;
      outputs[QuadState::StateIndexVel + j].weight = 10;
      outputs[QuadState::StateIndexVel + j].value[0] = target_vel.coeff(j);
    }
//   }
  controller_->step(outputs);
}

Path *HigherController::interceptForTarget(QuadState const& state) const {
  QuadraticIntercept intercept;
  intercept.state_ = state;
  intercept.max_linear_acceleration_ = MaxLinearAcceleration;
  intercept.max_pitch_acceleration_ = MaxPitchAcceleration;
  intercept.target_ = stateTargetToQuadratic();
  bool found;
  Path * path = intercept.interceptPath(&found);
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
