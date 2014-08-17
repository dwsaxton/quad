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
struct LinearQuad {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MatrixB B;
  QuadStateVector xstar;
};
LinearQuad linearize(Quad const *quad, double time_step);

class ControlledOutput {
public:
  ControlledOutput();
  double value;
  double weight;
};


class Controller {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Controller();

  Vector4d getPropInputs(QuadState const& state, const Vector4d& last_inputs, QVector<ControlledOutput> const& next);
  
private:
  void initSSC();
  void updateSSC(QuadState const& state, const Vector4d& last_inputs, const QVector< ControlledOutput >& outputs);
  SSC ssc_;
};

#endif
