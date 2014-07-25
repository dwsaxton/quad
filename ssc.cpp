#include "ssc.h"

#include <cassert>
#include <iostream>
using namespace std;

#include "qpsolver.h"

SSC::SSC() {
  n = l = 0;
}

void SSC::checkDimensions() {
  assert(x0.size() == n);
  assert(B.rows() == n);
  assert(B.cols() == l);
   
  if (E.rows() != 0) {
    assert(E.rows() == E0.rows());
    assert(E.cols() == l);
  }
}

VectorXd SSC::calc_du(VectorXd *predX) {
  checkDimensions();

  QP::Problem problem;
  problem.Phi = B.transpose() * Q * B + R;
  problem.phi = B.transpose() * Q.transpose() * (x0 - r);
  problem.Omega = E;
  problem.omega = E0;
  
  VectorXd du = QP::solve(problem);
  if (!isfinite(du[0])) {
    cout << "non-finite solution!" << endl;
    du.setZero();
  }
//    cout << "um="<<um.transpose()<<"  du="<<du.transpose()<<endl;
  
  if (predX) {
    *predX = x0 + B * du;
  }

  return du;
}

void testSsc() {
  SSC ssc;
  ssc.n = 2;
  ssc.l = 2;
  
  ssc.x0 = VectorXd(2);
  ssc.x0 << 1, 3;
  ssc.B = MatrixXd(2, 2);
  ssc.B << 2, 4, -1, -3;
  ssc.r = VectorXd(2);
  ssc.r << 0, 0;

  ssc.Q = MatrixXd(2, 2);
  ssc.Q << 1, 0, 0, 1;

  ssc.R = MatrixXd(2, 2);
  ssc.R.setZero();

  VectorXd du = ssc.calc_du();
  VectorXd x = ssc.x0 + ssc.B * du;
  assert(abs(x[0])  < 1e-5);
  assert(abs(x[1])  < 1e-5);
}
