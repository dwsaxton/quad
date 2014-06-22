#include "qpsolver.h"

#include "eiquadprog.h"
#include "EigenQP.h"

#include <cassert>
#include <iostream>
using namespace std;

using namespace QP;

bool Problem::dimensionsValid() const
{
   int n = Phi.rows();
   
   if ( Phi.cols() != n )
	  return false;
   
   if ( phi.rows() != n )
      return false;
   
   if ( phi.cols() != 1 )
      return false;
   
   // Do we have equality constrains?
   if ( H.rows() > 0 )
   {
      if ( H.cols() != Phi.cols() )
         return false;
      
      if ( H.rows() != h.rows() )
         return false;
   }
   
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

void test1() {
  
   Problem u;
   u.Phi.resize( 2, 2 );
   u.phi.resize( 2 );
   
   u.Phi << 1,-1,-1,2;
   u.phi << -2,-6;
   
   u.Omega.resize( 5,2 );
   u.omega.resize( 5 );
   u.Omega << 1,1,-1,2,2,1,-1,0,0,-1;
   u.omega << 2,2,3,0,0;
   
   VectorXd soln = solve(u);
   assert( abs(soln[0] - 2./3.) < 1e-6 );
   assert( abs(soln[1] - 4./3.) < 1e-6 );
}

#if 0
void test2() {
  Problem u;
  u.Phi.resize(2, 2);
  u.phi.resize(2);
  
  u.Phi << -10,0,0,-1;
  u.phi << 0,0;
  
  u.Omega.resize(4, 2);
  u.omega.resize(4);
  u.Omega << 1, 1, -1, 0, 0, -1, 1, 0;
  u.omega << 2, 0, 0, 1;
  
  cout << "Phi:" << endl << u.Phi << endl;
  cout << "Omega:" << endl << u.Omega << endl;
  
  VectorXd soln = solve(u);
  cout << soln.transpose() << endl;
  assert(abs(soln[0] - 1.0) < 1e-6);
  assert(abs(soln[1] - 1.0) < 1e-6);
}
#endif

void QP::test()
{
#if 0
   Problem u;
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
#endif
   
   test1();
//    test2();
}
VectorXd QP::solve( const Problem & qp )
{
   assert( qp.dimensionsValid() );
   
   MatrixXd G = qp.Phi;
   VectorXd g0 = qp.phi;
   MatrixXd CE = qp.H.transpose();
   VectorXd ce0 = -qp.h;
   MatrixXd CI = -qp.Omega.transpose();
   VectorXd ci0 = qp.omega;
   VectorXd x;
   
   Eigen::solve_quadprog(G,g0,CE,ce0,CI,ci0,x);
//    QP::solve_quadprog(G,g0,CE,ce0,CI,ci0,x);
   return x;
}

