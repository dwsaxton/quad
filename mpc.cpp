#include "mpc.h"

#include "qpsolver.h"

#include <QVector>

#include <iostream>
using namespace std;

Mpc::Mpc()
{
   n = l = m = 0;
   Ts = 0;
   Hp = Hu = Hw = 1;
   QIsDiagonal = false;
   permCValid = false;
}

Mpc::~Mpc()
{
//    AiB.resize(0);
//    delete K;
//    delete H;
//    E.resize(0);
//    H.resize(0,0);
}

void Mpc::checkDimensions()
{
   assert( f0.size() == n );
   assert( x0.size() == n );
   assert( um.size() == l );
   assert( Ac.rows() == n );
   assert( Ac.cols() == n );
   assert( Bc.rows() == n );
   assert( Bc.cols() == l );
   
   int in = l*Hu;
   int ref = m*(Hp-Hw+1);
   
//    cout << "r.size()="<<r.size()<<" m="<<m<<" ref="<<ref<<endl;
   assert( r.size() == ref );
   if ( permCValid )
	  assert( permC.size() == m );
   assert( C.rows() == m );
   assert( C.cols() == n );
   assert( Q.rows() == ref );
   assert( Q.cols() == ref );
   assert( R.rows() == in );
   assert( R.cols() == in );
   assert( d.rows() == n );
   
   // Sanity conditions for horizon values
   assert( Hw >= 1 );
   assert( Hp >= Hw );
   assert( Hp >= Hu );
   
   if ( E.rows() != 0 )
   {
	  assert( E.rows() == E0.rows() );
	  assert( E.cols() == in );
   }
   if ( F.rows() != 0 )
   {
	  assert( F.rows() == F0.rows() );
	  assert( F.cols() == in );
   }
   if ( G.rows() != 0 )
   {
	  assert( G.rows() == G0.rows() );
	  assert( G.cols() == Hp*m );
   }
}


VectorXd Mpc::calc_du( VectorXd *predX )
{
//   cout << "calc_du:" << endl;
//   cout << "r: " << r.transpose() << endl;
//   cout << "C: " << endl << C << endl;
//   cout << "Q: " << endl << Q << endl;

   // If haven't defined a disturbance then create a zero one
   if ( d.rows() == 0 )
   {
	  d.resize( n );
	  d.setZero();
   }
   
   checkDimensions();
   
   calc_JAB();
   calc_w();
   calc_sumAiB();
   calc_Dk();
   calc_CDk();
   calc_QP();
   calcConstraints();
   
//    cout << "A:"<<endl<<A<<endl<<"B:"<<endl<<B<<endl;
//    cout << "E:"<<endl<<E<<endl<<"F:"<<endl<<F<<endl;
   
   QP::Problem p;
   p.phi = phi;
   p.Phi = Phi;
   p.Omega = Omega;
   p.omega = omega;
   
   VectorXd DU = QP::solve(p);
   
   VectorXd du = DU.segment( 0, l );
   if ( !isfinite( du[0] ) )
   {
	  cout << "non-finite solution!" << endl;
	  du.setZero();
   }
//    cout << "um="<<um.transpose()<<"  du="<<du.transpose()<<endl;
   
   if ( predX )
   {
	  *predX = w.segment(0, n) + B * du - d;
   }
   
   return du;
}

void Mpc::calc_JAB()
{
   A = Ts * Ac;
   for ( int i = 0; i < n; ++i )
	  A(i,i) += 1;
   
   B = Ts * Bc;
   
   J = Ts*(f0 - Ac*x0 - Bc*um);
}

void Mpc::calc_w()
{
   VectorXd wprev = x0;
   VectorXd wnew( n );
   w.resize( n * Hp );
   VectorXd JdBu = J + d + B*um;
   
   for ( int i = 0; i < Hp; ++i )
   {
	  wnew = JdBu + A*wprev;
	  w.segment( i*n, n ) = wnew;
	  wprev = wnew;
   }
   
}

void Mpc::calc_sumAiB()
{
   sumAiB.resize( Hp+1 );
   sumAiB[0] = B;
   for ( int i = 1; i < Hp+1; ++i )
	  sumAiB[i] = A * sumAiB[i-1] + B;
}

void Mpc::calc_Dk()
{
   Dk.resize( Hp+1 );
   
   // Helper things
   MatrixXd D( n, l*Hu );
   
   for ( int k = 1; k <= Hp; ++k )
   {
	  D.setZero();
	  for ( int j = 0; j < Hu; ++j )
	  {
		 int i = k - j - 1;
		 if ( i >= 0)
			D.block( 0, j*l, n, l ) = sumAiB[i];
	  }
	  
	  Dk[k] = D;
   }
}

void Mpc::calc_CDk()
{
   CDk.resize( Hp+1 );
   
   for ( int k = 1; k <= Hp; ++k )
   {
	  if ( !permCValid )
		 CDk[k] = C * Dk[k];
	  else
	  {
		 CDk[k].resize( m, Dk[k].cols() );
		 for ( int i = 0; i < m; ++i )
		 {
// 			cout << i <<","<<permC[i]<<endl;
			CDk[k].row(i) = Dk[k].row( permC[i] );
		 }
	  }
   }
}

void Mpc::calc_QP()
{
   calc_K();
   calc_H();
   calc_phi();
   calc_Phi();
}

void Mpc::calc_K()
{
//    K = new VectorXd( m*(Hp-Hw+1) );
   K.resize( m*(Hp-Hw+1) );
   for ( int i = 0; i < Hp-Hw+1; ++i )
   {
	  K.segment( i*m, m ) = r.segment(i*m, m) - C * w.segment( n*(i+Hw-1), n );
   }
}

void Mpc::calc_H()
{
//    H = new MatrixXd( m*(Hp-Hw+1), l*Hu );
   H.resize( m*(Hp-Hw+1), l*Hu );
   
   for ( int i = 0; i < Hp-Hw+1; ++i )
   {
	  int k = i+Hw;
	  H.block( i*m, 0, m, l*Hu ) = CDk[k];
   }
}

void Mpc::calc_phi()
{
   if ( !QIsDiagonal )
   {
	  // Note in the following expression it is important that the bracketing is maintained
	  phi = -2 * H.transpose() * (Q * K);
   }
   else
   {
	  int s = K.rows();
	  VectorXd QK( s );
	  for ( int i = 0; i < s; ++i )
		 QK(i) = Q(i,i) * K(i);
	  
	  phi = -2 * H.transpose() * QK;
   }
}

void Mpc::calcHtQ() {
  int c = H.cols();
  int r = H.rows();
  if (HtQ.rows() != c || HtQ.cols() != r) {
    HtQ = MatrixXd(c, r);
  }
  for (int j = 0; j < r; ++j) {
    HtQ.col(j) = Q(j,j) * H.row(j).transpose();
  }
}

void Mpc::calc_Phi()
{
   if ( !QIsDiagonal )
	  Phi = 2*(H.transpose() * Q * H + R);
   else
   {
     calcHtQ();
	 Phi = 2*(HtQ * H + R);
   }
}

void Mpc::updateIdiag() {
  if (Idiag.rows() == l*Hu) {
    return;
  }
  Idiag = MatrixXd(l*Hu, l*Hu);
  Idiag.setZero();
  for ( int ii = 0; ii < Hu; ++ii )
  {
      for ( int jj = 0; jj <= ii; ++jj )
      {
        for ( int k = 0; k < l; ++k )
            Idiag( ii*l+k, jj*l+k ) = 1;
      }
  }
}

void Mpc::calcConstraints()
{
   // E
   MatrixXd OmegaE = E;
   VectorXd omegaE = E0;
   
   MatrixXd OmegaF;
   VectorXd omegaF;
   
   MatrixXd OmegaG;
   VectorXd omegaG;
   
   // F
   if ( F.rows() != 0 )
   {
     updateIdiag();
	  OmegaF = F * Idiag;
	  
	  VectorXd umconst( l*Hu );
	  for ( int i = 0; i < Hu; ++i )
		 umconst.segment( i*l, l ) = um;
	  omegaF = F0 - F * umconst;
   }
   
   
   // G
   if ( G.rows() != 0 )
   {
  
      if (DD.rows() != m*Hp || DD.cols() != l*Hu) {
        DD = MatrixXd(m*Hp, l*Hu);
      }
  
	  for ( int k = 1; k <= Hp; ++k ) {
		 DD.block( (k-1)*n, 0, m, l*Hu ) = CDk[k];
	  }
	  
	  OmegaG = G * DD;
	  omegaG = G0 - G * w;
   }
   
   const int cE = OmegaE.rows();
   const int cF = OmegaF.rows();
   const int cG = OmegaG.rows();
   
   Omega.resize( cE+cF+cG, l*Hu );
   omega.resize( cE+cF+cG );
   
   if ( cE > 0 )
   {
	  Omega.block( 0, 0, cE, l*Hu ) = OmegaE;
	  omega.segment( 0, cE ) = omegaE;
   }
   
   if ( cF > 0 )
   {
	  Omega.block( cE, 0, cF, l*Hu ) = OmegaF;
	  omega.segment( cE, cF ) = omegaF;
   }
   
   if ( cG > 0 )
   {
	  Omega.block( cE+cF, 0, cG, l*Hu ) = OmegaG;
	  omega.segment( cE+cF, cG ) = omegaG;
   }
}


void mpcTest()
{
   // Test a car, state (x,v), acceleration u, (dx/dt, dv/dt) = (v,u).
   
   // Initial starting state: At position 0, at rest.
   Vector2d x;
   x.setZero();
   
   VectorXd u(1);
   u.setZero();
   
   for ( int i = 0; i < 100; ++i )
   {
	  cout << endl << "x = "<<x.transpose()<<"  u = "<<u<<endl;
	  
	  Mpc m;
	  
	  Vector2d f0;
	  f0 << x[1],u[0];
	  m.f0 = f0;
	  
	  m.x0 = x;
	  m.um = u;
	  
	  Matrix2d Ac;
// 	  Ac << 0.1,1.1,-0.1,-0.05;
	  Ac << 0,1,0,0;
	  m.Ac = Ac;
	  
	  Vector2d Bc;
	  Bc << 0,1;
	  m.Bc = Bc;
	  
	  m.Ts = 0.1;
	  m.n = 2;
	  m.l = 1;
	  m.m = 2;
	  
	  m.Hp = 20;
	  m.Hu = 10;
	  m.Hw = 1;
	  
	  const int refl = m.Hp-m.Hw+1;
	  
	  VectorXd r( 2*refl );
	  for ( int i = 0; i < refl; ++i )
	  {
		 r[2*i] = 1; // position 1
		 r[2*i+1] = 0; // velocity 0
	  }
	  m.r = r;
	  
	  SparseMatrix<double> C(2,2);
	  C.setZero();
	  C.insert(0,0) = 1;
	  C.insert(1,1) = 1;
// 	  C << 1,0,0,1;
// 	  C << 1,0;
	  m.C = C;
	  
	  SparseMatrix<double> Q( 2*refl, 2*refl );
	  Q.setZero();
	  for ( int i = 0; i < refl; ++i )
	  {
		 Q.insert(2*i,2*i) = 1;
		 Q.insert(2*i+1,2*i+1) = 0.1;
	  }
	  m.Q = Q;
	  
	  MatrixXd R( m.Hu, m.Hu );
	  R.setZero();
	  for ( int i = 0; i < m.Hu; ++i )
		 R(i,i) = 1e-6;
	  m.R = R;
	  
	  MatrixXd F( 2*m.Hu, m.Hu );
	  VectorXd F0( 2*m.Hu );
	  F.setZero();
	  for ( int i = 0; i < m.Hu; ++i )
	  {
		 F( i, i ) = 1;
		 F0(i) = 1;
		 F( m.Hu+i, i ) = -1;
		 F0(i) = 1;
	  }
	  m.F = F;
	  m.F0 = F0;
	  
	  u += m.calc_du();
	  x[0] += m.Ts * x[1];
	  x[1] += m.Ts * u[0];
   }
}
