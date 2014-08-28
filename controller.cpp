#include "controller.h"

#include "controllooper.h"

using namespace QP;

#include <iostream>
using namespace std;

LinearQuad linearize(QuadState const& initial, const Vector4d& u0, double time_step) {
  VectorXd initial_vector = initial.toVector();
  Quad quad;
  quad.setState(initial);
  quad.setPropInput(u0);

  LinearQuad linear_quad;

  quad.step(time_step);
  linear_quad.xstar = quad.state().toVector();

  double eps = 1e-2;
  for (int i = 0; i < 4; ++i) {
    quad.setState(initial);
    quad.setPropInput(u0);
    quad.setPropInput(i, u0.coeff(i) + eps);
    quad.step(time_step);
    VectorXd newState = quad.state().toVector();
    for (int j = 0; j < QUAD_STATE_SIZE; ++j) {
      linear_quad.B(j, i) = (newState.coeff(j) - linear_quad.xstar.coeff(j)) / eps;
    }
  }

  return linear_quad;
}


//BEGIN class ControlledOutput
ControlledOutput::ControlledOutput() {
   weight = 0;
   value = 0;
}
//END class ControlledOutput


//BEGIN class Controller
Controller::Controller()
{
   initSSC();
}

Vector4d Controller::getPropInputs(const QuadState& state, const Vector4d& current_input, const QVector< ControlledOutput >& next) {
  // TODO re-implement having a drift / disturbance
  assert(next.size() == QUAD_STATE_SIZE);
  updateSSC(state, current_input, next);
  VectorXd predX; // TODO make use of this, or get rid of it
  VectorXd du = ssc_.calc_du(&predX);
  VectorXd newu = current_input + du;

  for ( int i = 0; i < 4; ++i )
  {
    if (!isfinite(newu[i])) {
      cout << "Warning: not finite!" << endl;
      newu[i] = 0;
    } else if (newu[i] < 0) {
      if (newu[i] < -1e-8) {
        cout << "newu["<<i<<"] = " << newu[i] << " < 0" << endl;
      }
      newu[i] = 0;
    } else if (newu[i] > 1) {
      if (newu[i] > 1 + 1e-8) {
        cout << "newu["<<i<<"] = " << newu[i] << " > 1" << endl;
      }
      newu[i] = 1;
    }
  }
  return newu;
}

void Controller::initSSC() {
   ssc_.n = QUAD_STATE_SIZE;
   ssc_.l = 4;
}

inline double double_min(double x, double y) {
  return x < y ? x : y;
}

void Controller::updateSSC(QuadState const& state, const Vector4d& current_input, const QVector< ControlledOutput >& outputs) {
  int QSS = QUAD_STATE_SIZE;

  LinearQuad linear_quad = linearize(state, current_input, TsWorldTarget());

  ssc_.x0 = linear_quad.xstar;
  ssc_.B = linear_quad.B;
  VectorXd r(QSS);
  MatrixXd Q(QSS, QSS);
  Q.setZero();
  for (int i = 0; i < QSS; ++i) {
    r[i] = outputs[i].value;
    Q(i, i) = outputs[i].weight;
  }
  ssc_.r = r;
  ssc_.Q = Q;
  
  // Small penalty for changes in the input?
  MatrixXd R(4, 4);
  R.setZero();
  for (int i = 0; i < 4; ++i) {
    R(i,i) = 1e-2;
  }
  ssc_.R = R;

  // Calculate constraints on propeller input
  double prop_start_time = 0.01; // time it takes for propellers to spin up or down to full speed TODO change this
  double max_abs_du = TsWorldTarget() / prop_start_time;
  MatrixXd E(8, 4);
  VectorXd E0(8);
  E.setZero();
  E0.setZero();
  for (int i = 0; i < 4; ++i) {
    E(2 * i, i) = 1;
    E(2 * i + 1, i) = -1;
    E0(2 * i) = double_min(max_abs_du, 1 - current_input[i]);
    E0(2 * i + 1) = double_min(max_abs_du, current_input[i]);
  }
  ssc_.E = E;
  ssc_.E0 = E0;
}
//END class Controller

