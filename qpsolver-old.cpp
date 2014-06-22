#include "qpsolver.h"

#include <Eigen/LU>
#include <Eigen/QR>

#include <cassert>
#include <iostream>
using namespace std;

using namespace QP;

bool Unconstrained::dimensionsValid() const
{
   int n = Phi.rows();
   
   //    if ( Phi.cols() != n )
   //       return false;
   
   if ( phi.rows() != n )
      return false;
   
   if ( phi.cols() != 1 )
      return false;
   
   return true;
}

bool LinConstrained::dimensionsValid() const
{
   if ( !Unconstrained::dimensionsValid() )
      return false;
   
   int n = Phi.rows();
   
   // Do we have equality constrains?
   if ( H.rows() > 0 )
   {
      if ( H.cols() != Phi.cols() )
         return false;
      
      if ( H.rows() != h.rows() )
         return false;
   }
   
   return true;
}


bool QuadConstrained::dimensionsValid() const
{
   if ( !LinConstrained::dimensionsValid() )
      return false;
   
   // Do we have inequality constraints?
   if ( Omega.rows() > 0 )
   {
      if ( Omega.rows() != omega.rows() )
         return false;
      
      if ( Omega.cols() != Phi.cols() )
         return false;
   }
   
   return true;
}


void QP::test()
{
   Unconstrained u;
   u.Phi.resize( 3, 2 );
   u.phi.resize( 3, 1 );
   
   // Check it works for systems where we have an exact soln
   u.Phi << 1,2,2,4,5,3;
   u.phi << 5,10,4;
   
   VectorXd soln = solve(u);
   assert( abs(soln[0] + 1) < 1e-6 );
   assert( abs(soln[1] - 3) < 1e-6 );
   
   // And another arbitrary example
   u.Phi << 8,4,2,6,1,3;
   u.phi << 8,0,6;
   soln = solve(u);
   assert( abs(soln[0] - 0.96) < 1e-6 );
   assert( abs(soln[1] - 0.08) < 1e-6 );
   
   // TODO test LinConstrained and QuadConstrained
}


VectorXd QP::solve( const Unconstrained & qp )
{
   assert( qp.dimensionsValid() );
   
   QR<MatrixXd> qr;
   qr.compute( qp.Phi );
   
   VectorXd QTb = qr.matrixQ().transpose() * qp.phi;
   
   MatrixXd R = qr.matrixR();
   
   VectorXd soln = R.marked<Eigen::UpperTriangular>().solveTriangular( QTb );
   
   return soln;
}


VectorXd QP::solve( const LinConstrained & qp )
{
   assert( qp.dimensionsValid() );
   
   if ( qp.H.rows() == 0 )
      return solve( Unconstrained(qp) );
   
   // TODO find P such that H P = ( H1 H2 ), H1 an n x r matrix
   MatrixXd P;
   int r = -1;
   
   int p = qp.H.rows();
   int n = qp.H.cols();
   
   MatrixXd H1 = qp.H.block( 0, 0, p, r );
   MatrixXd H2 = qp.H.block( 0, r, p, n-r );
   
   QR<MatrixXd> qr;
   qr.compute( H1 );
   
   // Matrices such that Q^T H P = ( (R11 R12) (0 0) )   ( r rows, p-r rows )
   MatrixXd Q = qr.matrixQ();
   MatrixXd R11 = qr.matrixR();
   MatrixXd R12 = Q.transpose() * H2;
   
   MatrixXd R11inv;
   R11.marked<Eigen::UpperTriangular>().computeInverse( &R11inv );
   
   VectorXd db = Q.transpose() * qp.h;
   VectorXd d1b = db.block( 0, 0, r, 1 );
   VectorXd d2b = db.block( r, 0, p-r, 1 );
   
   MatrixXd Ab = qp.H * P;
   MatrixXd A1b = Ab.block( 0, 0, p, r );
   MatrixXd A2b = Ab.block( 0, r, p, n-r );
   
   MatrixXd A2h = A2b - A1b * R11inv * R12;
   
   VectorXd bh = qp.h - A1b * R11inv * d1b;
   
   Unconstrained u;
   u.Phi = A2h;
   u.phi = bh;
   
   VectorXd x2b = solve( u );
   VectorXd x1b = R11inv * ( d1b - R12*x2b );
   
   VectorXd xb( n );
   xb.block( 0, 0, r, 1 ) = x1b;
   xb.block( r, 0, n-r, 1 ) = x2b;
   
   return P * xb;
}


VectorXd QP::solve( const QuadConstrained & qp )
{
   VectorXd init = findFeasiblePoint( qp );
   return solve( qp, init );
}


VectorXd QP::solve( const QuadConstrained & qp, const VectorXd & init )
{
   (void)qp;
   (void)init;
   return VectorXd();
}


void QP::iterate( ActiveSet *as )
{
   (void)as;
}


VectorXd QP::findFeasiblePoint( const QuadConstrained & qp )
{
   (void)qp;
   return VectorXd();
}

