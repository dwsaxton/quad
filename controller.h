#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QObject>
#include <QVector>

#include "ssc.h"
#include "quad.h"

typedef Matrix<double, QUAD_STATE_SIZE, QUAD_STATE_SIZE> MatrixA;
typedef Matrix<double, QUAD_STATE_SIZE, 4> MatrixB;

namespace QP { class Problem; }

/**
 * Linearized state of the quadrocopter.
 */
struct LinearQuad
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  MatrixB B;
  QuadStateVector xstar;
};
LinearQuad linearize(const QuadState & initial, const Vector4d & u0, double time_step);

class ControlledOutput
{
public:
  ControlledOutput();
  double value;
  double weight;
};


class Controller {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Controller();

  void step(QVector<ControlledOutput> const& next);
  
private:
  void initSSC();
  void updateSSC(QVector<ControlledOutput> const& next);

  VectorXd m_predX;
  
  // Previous disturbance
  VectorXd m_pred_d;
  
  // Previously applied input
  Vector4d m_prev_u;
  
  SSC ssc_;
};

#endif
