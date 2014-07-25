#ifndef SSC_H
#define SSC_H

#include <Eigen/Geometry>
using namespace Eigen;

void testSsc();

/**
 * "Single step" controller. Simplified version of MPC. Only do one step, so
 * controls are easier to specify. We have a system of the form
 *
 *         x = x_0 + B du,
 *
 * where x_new is the new value of some state x, du is the change in inputs,
 * B is some matrix, and x_0 is the state that x would be if there were no
 * change in inputs.
 *
 * This class calculates the optimal value of du, subject to the constraint
 *
 *        E du <= E_0
 *
 * and a cost function
 *
 *       ||x_new  - r||^2_Q   +  ||du||^2_R
 */
class SSC {
public:
  SSC();
   
  VectorXd x0;
  MatrixXd B;

  /**
  * n is the dimension of x and x_0, the number of variables in the system.
  */
  int n;
  /**
  * l is the dimension of u, the number of controls.
  */
  int l;
  /**
  * Reference trajectory for x.
  * Vector of length m*(Hp-Hw+1).
  */
  VectorXd r;
  /**
  * Penalize deviations from the reference trajectory.
  */
  MatrixXd Q;
  /**
  * Penalize changes in the input.
  * Square matrix with l*Hu rows.
  */
  MatrixXd R;
  /**
  * Constraint on control input change:
  * E du <= E0
  */
  MatrixXd E;
  /**
  * Constraint on control input change:
  * E du <= E0
  */
  VectorXd E0;

  /**
  * Calculate the optimal change in input u to apply. This function also
  * returns what it expects the next x value to be in predX (minus the current
  * correction for disturbance), used for disturbance correction.
  */
  VectorXd calc_du( VectorXd *predX = 0 );

private:
  void checkDimensions();
};

#endif // SSC_H
