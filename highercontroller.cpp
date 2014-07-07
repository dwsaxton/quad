#include "highercontroller.h"

#include "controller.h"
#include "quadraticintercept.h"
#include "rotationplanner.h"
#include "world.h"

HigherController::HigherController() {
  have_control_ = false;
  controller_ = new Controller();
  target_pos_ = Vector3d(0, 0, 12);
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
  Quadratic3d intercept = interceptForTarget(state);

  // TODO this should be done in other code
  double intercept_deriv = intercept.b.norm();
  Vector3d target_pos = intercept.eval(time_step / intercept_deriv);
  Vector3d target_vel = (target_pos - state.pos) / time_step; // TODO completely wrong way of doing it

  // Calculate the difference in angle between our current orientation, and that required to follow
  // the required trajectory
  Vector3d current_z = state.orient.conjugate() * Vector3d(0, 0, 1);
  Vector3d target_z = intercept.b.normalized();
  Quaterniond offset_rotation;
  offset_rotation.setFromTwoVectors(current_z, target_z); // TODO is this the correct way around?
  double angle = acos(current_z.dot(target_z));

  QVector<ControlledOutput> outputs(QUAD_STATE_SIZE);

  RotationPlanner rotation_planner;
  rotation_planner.current_orientation_ = state.orient;
  rotation_planner.current_omega_ = state.omega;
  rotation_planner.max_pitch_acceleration_ = 4; // TODO see below too
  rotation_planner.target_orientation_ =  offset_rotation * state.orient; // TODO check this is the correct order for rotation

  Quaterniond next_orient;
  Vector3d next_omega;
  rotation_planner.calcNextStep(time_step, &next_orient, &next_omega);
  for (int j = 0; j < 4; ++j) {
    outputs[QuadState::StateIndexOrient + j].used = true;
    outputs[QuadState::StateIndexOrient + j].weight = 1;
    outputs[QuadState::StateIndexOrient + j].value[0] = next_orient.coeffs()[j];
  }
  for (int j = 0; j < 3; ++j) {
    outputs[QuadState::StateIndexomega + j].used = true;
    outputs[QuadState::StateIndexomega + j].weight = 1;
    outputs[QuadState::StateIndexomega + j].value[0] = next_omega.coeff(j);
  }

  if (angle < M_PI/4 /* TODO used elsewhere too, shouldn't be hard-coded */) {
    for (int j = 0; j < 3; ++j) {
      outputs[QuadState::StateIndexPos + j].used = true;
      outputs[QuadState::StateIndexPos + j].weight = 10;
      outputs[QuadState::StateIndexPos + j].value[0] = target_pos.coeff(j);
      outputs[QuadState::StateIndexVel + j].used = true;
      outputs[QuadState::StateIndexVel + j].weight = 10;
      outputs[QuadState::StateIndexVel + j].value[0] = target_vel.coeff(j);
    }
  }
  controller_->step(outputs);
}

Quadratic3d HigherController::interceptForTarget(QuadState const& state) const {
  QuadraticIntercept intercept;
  intercept.state_ = state;
  intercept.max_linear_acceleration_ = 15; // TODO get the correct value!
  intercept.max_pitch_acceleration_ = 4; // TODO get the correct value!
  intercept.target_ = stateTargetToQuadratic();
  bool success;
  return intercept.interceptPath(&success);
}

Quadratic3d HigherController::stateTargetToQuadratic() const {
  Quadratic3d quadratic;
  quadratic.c = target_pos_;
  quadratic.a = Vector3d(0, 0, 0.5 * 9.81);
  return quadratic;
}
