#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QObject>
#include <QVector>

#include "mpc.h"
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
  
  MatrixA Ac;
  MatrixB Bc;
  QuadStateVector f0;
  QuadStateVector x0;
  Vector4d um;
};
LinearQuad linearize( const QuadState & qsx0, const Vector4d & u0 );


class ControlledOutput
{
public:
  ControlledOutput();
  
  void reset();
  
  bool used;
  QVector<double> value;
  double weight;
  void setConstValue( double v );
};


class Controller {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Controller();

  void step(QVector<ControlledOutput> const& next);
  
private:
  void initMpc();
  void updateMpc(QVector<ControlledOutput> const& next);
  void addPropConstraints();

  VectorXd m_predX;
  
  // Previous disturbance
  VectorXd m_pred_d;
  
  // Previously applied input
  Vector4d m_prev_u;
  
  Mpc m_mpc;
};

#endif
