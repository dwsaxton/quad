#include "observer.h"

#include "quad.h"
#include "sensors.h"
#include "world.h"

#include <iostream>
using namespace std;

// #include <cstdio>
#include <cstdlib>

#include <Eigen/Cholesky>
#include <Eigen/Core>

//BEGIN class ObserverState
ObserverState::ObserverState( const VectorXd & asVector )
{
   fromVector(asVector);
}
ObserverState::ObserverState( const QuadState & qs )
: QuadState(qs)
{
   resetBiases();
}
ObserverState::ObserverState()
{
   reset();
}
void ObserverState::reset()
{
   QuadState::reset();
   resetBiases();
}
void ObserverState::resetBiases()
{
   ba.setZero();
   bg.setZero();
   bgps.setZero();
}

ObserverStateVector ObserverState::toVector() const
{
   ObserverStateVector v;
   v.segment( 0, QUAD_STATE_SIZE ) = QuadState::toVector();
   v.segment( QUAD_STATE_SIZE, 3 ) = ba;
   v.segment( QUAD_STATE_SIZE+3, 3 ) = bg;
   v.segment( QUAD_STATE_SIZE+6, 3 ) = bgps;
   return v;
}

void ObserverState::fromVector( const ObserverStateVector & v )
{
   QuadState::fromVector( v.segment( 0, QUAD_STATE_SIZE ) );
   ba = v.segment( QUAD_STATE_SIZE, 3 );
   bg = v.segment( QUAD_STATE_SIZE+3, 3 );
   bgps = v.segment( QUAD_STATE_SIZE+6, 3 );
}
//END class ObserverState



//BEGIN class Observer
const int L = OBSERVER_STATE_SIZE;

Observer::Observer()
{
   m_P.resize(OBSERVER_STATE_SIZE,OBSERVER_STATE_SIZE);
   reset();
}

void Observer::reset()
{
   m_xhat.reset();
   m_preUpdateState.reset();
   m_P.setZero();
}

void add_vvTs( MatrixOS &m, VectorXd *x )
{
   // The inner most loop of our calculation operation is called O(L^3) times,
   // so at least 10^6 times a second at the time of writing this comment.
   // Accessing eigen methods seems to have too much overhead. (perhaps this
   // disappears when optimizing turned on?) Anyway, much faster to convert
   // to native array operations.
   
   double y[2*L+1][L];
   for ( int i = 0; i < 2*L+1; ++i )
   {
      for ( int j = 0; j < L; ++j )
         y[i][j] = x[i][j];
   }
   
   for ( int i = 0; i < L; ++i )
   {
      for ( int j = 0; j <= i; ++j )
      {
         double r = 0;
         for ( int k = 0; k < 2*L+1; ++k )
            r += y[k][i]*y[k][j];
         m(i,j) = m(j,i) = r;
      }
   }
}

void Observer::step()
{
//    stepSensors();
//    predict();
   m_preUpdateState = m_xhat;
//    update();
   
//    m_xhat.pos.setZero();
//    m_xhat.orient.normalize();
   m_xhat = World::self()->simulatedQuad()->state();
}

const double kappa = 2;

void Observer::stepSensors()
{
//    cout << "################" << endl;
//    cout << "Before doing random walk, old covariance matrix is" << endl << m_P << endl;
   
   VectorXd scale(L);
   for ( int i = 0; i < L; ++i )
	  scale(i) = 1;
   VectorXd add(L);
   add.setZero();
   
   Vector3d bias_scale, bias_add;
   
   Sensors *s = World::self()->sensors();
   
   double tsWorld = TsWorldTarget();
   
   m_a = s->readAccelerometer( tsWorld, &bias_scale, &bias_add );
   scale.segment( QUAD_STATE_SIZE, 3 ) = bias_scale;
   add.segment( QUAD_STATE_SIZE, 3 ) = bias_add;
   
   m_g = s->readGyroscope( tsWorld, &bias_scale, &bias_add );
   scale.segment( QUAD_STATE_SIZE+3, 3 ) = bias_scale;
   add.segment( QUAD_STATE_SIZE+3, 3 ) = bias_add;
   
   m_gps = s->readGPS( tsWorld, &bias_scale, &bias_add );
   scale.segment( QUAD_STATE_SIZE+6, 3 ) = bias_scale;
   add.segment( QUAD_STATE_SIZE+6, 3 ) = bias_add;
   
   for ( int i = 0; i < L; ++i )
   {
	  for ( int j = 0; j < L; ++j )
	  {
		 m_P(i,j) *= scale(i);
		 m_P(i,j) *= scale(j);
	  }
   }
   
   for ( int i = 0; i < L; ++i )
	  m_P(i,i) += add(i);
   
//    cout << "After doing random walk, new covariance matrix is" << endl << m_P << endl;
}

void Observer::predict()
{
   genSigmaPoints();
   propSigmaPoints();
   estimateStateCov();
}

MatrixOS Observer::sqrtP()
{
   // Decompose m_P as m_P = P^T L D L^T P
   //  where P = permutation matrix
   //        L = lower triangular with unit diagonal
   //        D = diagonal matrix with zeroes in the bottom rank(m_P)-n submatrix
   LDLT<MatrixOS> ldlt = m_P.ldlt();
   
   Transpositions<Dynamic> P = ldlt.transpositionsP();
   MatrixOS L = ldlt.matrixL();
   
   Diagonal<const Matrix<double,OBSERVER_STATE_SIZE,OBSERVER_STATE_SIZE>,0> D = ldlt.vectorD();
   DiagonalMatrix<double,OBSERVER_STATE_SIZE> sqrtD;
   sqrtD.setZero();
   for ( int i = 0; i < OBSERVER_STATE_SIZE; ++i )
   {
      double s = D(i);
      sqrtD.diagonal()(i) = (s<=0) ? 0 : sqrt(s);
   }
   
   return P.transpose() * L * sqrtD;
}


void Observer::genSigmaPoints()
{
   const int L = OBSERVER_STATE_SIZE;
   
   MatrixOS sqrtLlP = sqrt(L + kappa) * sqrtP();
   
   m_chi0 = ObserverState(m_xhat).toVector();
   
   for ( int i = 0; i < L; ++i )
   {
	  m_chii[i] = m_chi0 + sqrtLlP.col(i);
	  m_chii[i+L] = m_chi0 - sqrtLlP.col(i);
   }
}

void Observer::propSigmaPoints()
{
   m_newchi0 = f( m_chi0 ).toVector();
   for ( int i = 0; i < 2*L; ++i )
   {
	  m_newchii[i] = f( m_chii[i] ).toVector();
   }
}

void Observer::estimateStateCov()
{
   double Ws0 = kappa / (L + kappa);
   double Wc0 = Ws0;
   double Wsi, Wci;
   Wsi = Wci = 0.5 / (L + kappa);
   
   VectorXd newx( OBSERVER_STATE_SIZE );
   newx.setZero();
   
//    newx = m_newchi0;
   newx += Ws0 * m_newchi0;
   for ( int i = 0; i < 2*L; ++i )
	  newx += Wsi * m_newchii[i];
   
   ObserverState y;
   y.fromVector(newx);
   m_xhat = y;
   
   VectorXd chil[2*L+1];
   chil[0] = sqrt(Wc0)*(m_newchi0 - newx);
   for ( int i = 0; i < 2*L; ++i )
      chil[i+1] = sqrt(Wci)*(m_newchii[i] - newx);
   
   add_vvTs( m_P, chil );
}

void Observer::update()
{
   MatrixXd H1( 3, QUAD_STATE_SIZE );
   MatrixXd H2( 3, 9 );
   H1.setZero();
   H2.setZero();
   for ( int i = 0; i < 3; ++i )
   {
	  H1(i, i) = 1;
	  H2(i,ObserverState::StateIndex_bgps+i) = 1;
   }
   
   const int sensors = OBSERVER_STATE_SIZE-QUAD_STATE_SIZE;
   
   MatrixXd sxx = m_P.block( 0, 0, QUAD_STATE_SIZE, QUAD_STATE_SIZE );
   MatrixXd sxb = m_P.block( 0, QUAD_STATE_SIZE, QUAD_STATE_SIZE, sensors );
   MatrixXd sbx = sxb.transpose();
   MatrixXd sbb = m_P.block( QUAD_STATE_SIZE, QUAD_STATE_SIZE, sensors, sensors );
   
   MatrixXd A = H1 * sxx * H1.transpose() + H2 * sbb * H2.transpose() - H1 * sxb * H2.transpose() - H2 * sbx * H1.transpose();
   MatrixXd B = sxb * H2.transpose() - sxx * H1.transpose();
   
   // Optimal Kalman gain
   MatrixXd L = B * A.inverse();
   
   VectorXd xhat = m_xhat.toVector();
   
   VectorXd z = H1 * xhat - m_gps;
//    cout << "z="<<z.transpose()<<endl;
   
   xhat += L * z;
   m_xhat.fromVector( xhat );
   
   // The matrix I + L H_1
   MatrixXd ILH1 = L*H1;
   for ( int i=0; i<ILH1.rows(); ++i )
      ILH1(i,i) += 1;
   
   // The matrix L H_2
   MatrixXd LH2 = L*H2;
   
   MatrixXd t = ILH1*sxb*LH2.transpose();
   m_P.block( 0, 0, QUAD_STATE_SIZE, QUAD_STATE_SIZE ) = ILH1 * sxx * ILH1.transpose() + LH2 * sbb * LH2.transpose() - t - t.transpose();
   
   MatrixXd nsxb = ILH1*sxb - LH2*sbb;
   
   m_P.block( 0, QUAD_STATE_SIZE, QUAD_STATE_SIZE, sensors) = nsxb;
   m_P.block( QUAD_STATE_SIZE, 0, sensors, QUAD_STATE_SIZE ) = nsxb.transpose();
}

ObserverState Observer::f( const ObserverState & x0 )
{
   QuadState state( x0 );
   
   // Remove biases
   Vector3d a = m_a - x0.ba;
   Vector3d g = m_g - x0.bg;
   
   // Remove gravity
   a -= 9.81 * state.rotateSpaceToBody( ez );
   
//    a.setZero();
   
   // Best estimate of actual omega
   state.omega = g;
   
   state = integrate( state, a, TsWorldTarget() );
   
   ObserverState x1( state );
   x1.ba = x0.ba;
   x1.bg = x0.bg;
   x1.bgps = x0.bgps;
   
   return x1;
}

QuadState Observer::integrate( QuadState state, const Vector3d & a, double t )
{
   state.calcDeriv( &k1, a );
   state += k1 * t;
   state.orient.normalize();
   return state;
}
//END class Observer
