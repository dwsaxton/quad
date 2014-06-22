#ifndef QPSOLVER_H
#define QPSOLVER_H

#include <Eigen/Geometry>
using namespace Eigen;

#include <QVector>

// Solve
//   min 1/2 x^T Phi x + phi^T x
// 
// subject to
//   H x = h
// and
//   Omega x <= omega

namespace QP {

class Problem
{
   public:  
      MatrixXd Phi;
      VectorXd phi;
      MatrixXd H;
      VectorXd h;
      MatrixXd Omega;
      VectorXd omega;
      
      virtual bool dimensionsValid() const;
};


void test();

VectorXd solve( const Problem & qp );

};

#endif
