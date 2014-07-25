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
  QP::Problem problem;
  problem.Phi = B.transpose() * Q * B + R;
  problem.phi = B.transpose() * Q.transpose() * (x0 - r);
  problem.Omega = E;
  problem.omega = E0;
  
  VectorXd du = QP::solve(problem);
  if (!isfinite(du[0]))
  {
    cout << "non-finite solution!" << endl;
    du.setZero();
  }
//    cout << "um="<<um.transpose()<<"  du="<<du.transpose()<<endl;
  
  if (predX) {
    *predX = x0 + B * du;
  }

  return du;
}

