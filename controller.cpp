#include "controller.h"

#include "mpc.h"
#include "observer.h"
#include "world.h"

#include <QVector>

using namespace QP;

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <cassert>
#include <cmath>
#include <iostream>
using namespace std;

LinearQuad linearize( const QuadState & qsx0, const Vector4d & u0 )
{
   QuadModel model;
   
//    // How does automatic control handle an imperfect model?
//    model.M *= 0.9;
//    model.c *= 1.1;
//     model.cF *= 1.2;
//    model.g = 0;
//    model.M = 1;
//    model.c = Vector3d( 0.01, 0, -0.05 );
   
   LinearQuad lq;
   
   lq.x0 = qsx0.toVector();
   lq.um = u0;
   
   QuadState d0;
   QuadState d2;
   QuadStateVector d2v;
   
   model.calcDeriv( &d0, qsx0, u0 );
   lq.f0 = d0.toVector();
   
   double epsilon = 0.01;
   
   lq.Ac.resize( QUAD_STATE_SIZE, QUAD_STATE_SIZE );
   lq.Bc.resize( QUAD_STATE_SIZE, 4 );
   
   QuadStateVector d;
   
   QuadState x2;
   QuadStateVector x2v;
   for ( int j = 0; j < QUAD_STATE_SIZE; ++j )
   {
      x2v = lq.x0;
      x2v[j] += epsilon;
      x2.fromVector( x2v );
      
      model.calcDeriv( &d2, x2, u0 );
      
	  d2v = d2.toVector();
	  
      d = (d2v - lq.f0) / epsilon;
      
      for ( int i = 0; i < QUAD_STATE_SIZE; ++i )
         lq.Ac( i, j ) = d[i];
   }
   
   Vector4d u2;
   for ( int j = 0; j < 4; ++j )
   {
      u2 = u0;
      u2[j] += epsilon;
      
      model.calcDeriv( &d2, qsx0, u2 );
      
      d = (d2.toVector() - lq.f0) / epsilon;
      
      for ( int i = 0; i < QUAD_STATE_SIZE; ++i )
         lq.Bc( i, j ) = d[i];
   }
   
   return lq;
}


//BEGIN class ControlledOutput
ControlledOutput::ControlledOutput()
{
   reset();
}

void ControlledOutput::reset()
{
   used = false;
   weight = 1;
   
   value.resize(0);
   value.resize(CONTROLLER_HP);
}

void ControlledOutput::setConstValue( double v )
{
  for ( int i = 0; i < value.size(); ++i )
	 value[i] = v;
}
//END class ControlledOutput


//BEGIN class Controller
Controller::Controller()
{
//    m_haveControl = false;
//    m_outputs.resize( QUAD_STATE_SIZE );
   initMpc();
}

void Controller::step(const QVector< ControlledOutput >& next)
{
  assert(next.size() == QUAD_STATE_SIZE);
   m_prev_u = World::self()->simulatedQuad()->propInput();
   updateMpc(next);
   VectorXd du = m_mpc.calc_du( &m_predX );
   VectorXd newu = m_prev_u + du;

   for ( int i = 0; i < 4; ++i )
   {
     if (!isfinite(newu[i])) {
       cout << "Warning: not finite!" << endl;
       newu[i] = 0;
     } else if (newu[i] < 0) {
       cout << "newu["<<i<<"] = " << newu[i] << " < 0" << endl;
       newu[i] = 0;
     } else if (newu[i] > 1) {
       cout << "newu["<<i<<"] = " << newu[i] << " > 1" << endl;
       newu[i] = 1;
     }
   }
   World::self()->setQuadInput( newu );

   m_pred_d = m_mpc.d;
   m_prev_u = newu;
}


// bool isPointingBadly(const QuadState &state) {
//    double z = state.rotateBodyToSpace(ez)(2);
//    return z < 0;
// }

// bool isSpinningTooFast(const QuadState &state) {
//   return state.omega.norm() > 10;
// }

// void Controller::updateControlledOutputs( const QuadState & state )
// {
//   if (isSpinningTooFast(state)) {
//     updateToStopSpin(state);
//   }
//   else {
//     updateControlledOrientation(state);
//     if (!isPointingBadly(state)) {
//       updateControlledPosition(state);
//     } else {
//       cout << "Pointing Badly!" << endl;
//     }
//   }
// }

// void Controller::updateControlledOrientation( const QuadState & state )
// {
//   bool correct = !isPointingBadly(state);
//    // Try to point it upwards
//    Quaterniond q0 = state.orient;
//    double yaw, pitch, roll;
//    GetEulerAngles(q0, yaw, pitch, roll);
//    double zRotn = yaw;
//    if (!correct)
//      cout << "zRotn="<<zRotn<<endl;
//    Quaterniond q1( AngleAxisd( zRotn, ez ) );
//    double z = state.bodyToSpace(ez)(2);
//    double angle = acos(z);
//    if (!correct)
//      cout << "q0: yaw="<<yaw<< " pitch="<<pitch<< " roll="<<roll << endl;
//    GetEulerAngles(q1, yaw, pitch, roll);
//    if (!correct)
//      cout << "q1: yaw="<<yaw<< " pitch="<<pitch<< " roll="<<roll << endl;
//    
//    double weight = 1;
//    
//    for ( int j = 0; j < 4; ++j )
//    {
// 	  m_outputs[QuadState::StateIndexOrient + j].used = true;
// 	  m_outputs[QuadState::StateIndexOrient + j].weight = weight;
//    }
//    
//    for ( int i = 0; i < CONTROLLER_HP; ++i )
//    {
// 	  double speed = 2; // orientation speed
// 	  double t = (i+1)*TsControllerTarget();
// 	  double totalTime = abs(angle) / speed;
// 	  double prop = t / totalTime;
// 	  if ( prop > 1 )
// 		 prop = 1;
// 	  
// 	  Quaterniond q = q0.slerp( prop, q1 );
//       GetEulerAngles(q, yaw, pitch, roll);
//       if (!correct)
//         cout << " q("<<i<<"): yaw="<<yaw<< " pitch="<<pitch<< " roll="<<roll << endl;
// 	  
// 	  for ( int j = 0; j < 4; ++j )
// 		 m_outputs[QuadState::StateIndexOrient + j].value[i] = q.coeffs()(j);
//    }
// 
// 
//    // velocity decrease per time-step
//    double dv = 10 * TsControllerTarget();
//    for ( int i = 0; i < 3; ++i )
//    {
//       m_outputs[QuadState::StateIndexVel+i].used = true;
//       m_outputs[QuadState::StateIndexomega+i].used = true;
// //       m_outputs[QuadState::StateIndexPos+i].used = true;
//       m_outputs[QuadState::StateIndexVel+i].weight = weight;
//       m_outputs[QuadState::StateIndexomega+i].weight = weight;
// //       m_outputs[QuadState::StateIndexPos+i].weight = weight;
//       for (int j = 0; j < CONTROLLER_HP; ++j) {
//         m_outputs[QuadState::StateIndexPos+i].value[j] = state.pos.coeff(i); // maintain current position
//         double v = state.vel[i];
//         double jdv = dv * (j+1);
//         if (abs(jdv) > abs(v)) {
//           v = 0;
//         } else {
//           v -= (v > 0) ? jdv : -jdv;
//         }
//         m_outputs[QuadState::StateIndexVel+i].value[j] = v;
//       }
//    }
// }

// void Controller::updateToStopSpin(const QuadState &state) {
//    for ( int i = 0; i < 3; ++i ) {
//       m_outputs[QuadState::StateIndexomega+i].used = true;
//       m_outputs[QuadState::StateIndexomega+i].weight = 1;
//       for (int j = 0; j < CONTROLLER_HP; ++j) {
//         m_outputs[QuadState::StateIndexomega+i].value[j] = state.omega[i] + (state.omega[i] > 0 ? -(j+1) : (j+1));
//       }
//    }
// }

// void Controller::updateControlledPosition( const QuadState & state )
// {
//    // Position
//    m_outputs[QuadState::StateIndexPos+0].used = true;
//    m_outputs[QuadState::StateIndexPos+0].weight = 20;
//    m_outputs[QuadState::StateIndexPos+1].used = true;
//    m_outputs[QuadState::StateIndexPos+1].weight = 20;
//    m_outputs[QuadState::StateIndexPos+2].used = true;
//    m_outputs[QuadState::StateIndexPos+2].weight = 20;
//    
//    Vector3d targetPos;
//    targetPos << 0, 0, 12;
//    int maxSpeed = 20;
//    Vector3d offset = targetPos - state.pos;
//    double distanceToTarget = offset.norm();
//    double timeToTarget = distanceToTarget / maxSpeed;
//    
//    for ( int i = 0; i < CONTROLLER_HP; ++i )
//    {
//      double t = (i+1) * TsControllerTarget();
//      double prop = min(1.0, t / timeToTarget);
//      Vector3d instantTarget = state.pos + prop * offset;
//      for ( int coord = 0; coord < 3; ++coord )
//      {
//        m_outputs[QuadState::StateIndexPos+coord].value[i] = instantTarget.coeff(coord);
//      }
//    }
// }

void Controller::initMpc() {
   m_mpc.n = QUAD_STATE_SIZE;
   m_mpc.l = 4;
   m_mpc.Hp = CONTROLLER_HP;
   m_mpc.Hu = CONTROLLER_HU;
   m_mpc.Hw = 1;
   addPropConstraints();
}
  
void Controller::updateMpc(QVector<ControlledOutput> const& outputs)
{
   int QSS = QUAD_STATE_SIZE;

   Vector4d um = m_prev_u;
   QuadState state = World::self()->observer()->state();
   LinearQuad lq = linearize( state, um );
   
//    for ( int i = 0; i < QSS; ++i )
// 	  m_outputs[i].reset();
//    updateControlledOutputs( state );
   
   int numOutputs = 0;
   for ( int i = 0; i < QSS; ++i )
   {
	  if ( outputs[i].used )
		 numOutputs++;
   }
   
   m_mpc.f0 = lq.f0;
   m_mpc.x0 = lq.x0;
   m_mpc.um = lq.um;
   m_mpc.m = numOutputs;
   m_mpc.Ac = lq.Ac;
   m_mpc.Bc = lq.Bc;
   m_mpc.Ts = TsControllerTarget();
   
   // The function updateControlledOutputs assumes Hw = 1
   assert( m_mpc.Hw == 1 );
   
   int refl = m_mpc.Hp - m_mpc.Hw + 1;
   
   VectorXd r( refl * numOutputs );
   r.setZero();
   int at = 0;
   for ( int i = 0; i < refl; ++i )
   {
	  for ( int j = 0; j < QSS; ++j )
	  {
		if ( !outputs[j].used )
			continue;
		r[at] = outputs[j].value[i];
		++at;
	  }
   }
   m_mpc.r = r;
   
   SparseMatrix<double> C( numOutputs, QSS );
   C.setZero();
   m_mpc.permC.resize( numOutputs );
   at = 0;
   for ( int j = 0; j < QSS; ++j )
   {
	  if ( !outputs[j].used )
		 continue;
	  m_mpc.permC[at] = j;
	  C.insert(at, j) = 1;
	  ++at;
   }
   m_mpc.C = C;
   m_mpc.permCValid = true;
   
   // Penalize deviations from the reference position uniformly
   m_mpc.Q.resize( numOutputs*refl, numOutputs*refl );
   // WARNING don't call Q.setZero, it takes up a lot of time, we don't need to.
//    m_mpc.Q.setZero();
   at = 0;
   m_mpc.QIsDiagonal = true;
   for ( int i = 0; i < refl; ++i )
   {
	  for ( int j = 0; j < QSS; ++j )
	  {
		 if ( !outputs[j].used )
			continue;
		 m_mpc.Q(at,at) = outputs[j].weight;
		 ++at;
	  }
   }
   
   // Small penalty for changes in the input?
   MatrixXd R( 4*m_mpc.Hu, 4*m_mpc.Hu );
   R.setZero();
   for ( int i = 0; i < 4*m_mpc.Hu; ++i )
	  R(i,i) = 1e-2;
   m_mpc.R = R;
   
   // Finally update the disturbance estimate
   if ( m_predX.rows() == m_mpc.n )
   {
// 	  VectorXd new_d = lq.x0 - m_predX;
	  // Rather than use the observer predicted state that we pass to the MPC,
	  // we use instead the state which has been predicted using accelerometer
	  // measurements rather than GPS measurements.
	  VectorXd lastApproxState = World::self()->observer()->preUpdateState().toVector();
	  VectorXd new_d = lastApproxState - m_predX;
	  double alpha = 1 * TsControllerTarget();
	  // Smooth out estimate of disturbance; our observed state won't be quite accurate..
	  m_mpc.d = (1-alpha) * m_pred_d + alpha * new_d;
   }
}

void Controller::addPropConstraints()
{
   // Bound how quickly the propeller speed can change
   MatrixXd E( 2*4*m_mpc.Hu, 4*m_mpc.Hu );
   E.setZero();
   VectorXd E0( 2*4*m_mpc.Hu );
   E0.setZero();
   
   // Bound propeller input between 0 and 1
   MatrixXd F( 2*4*m_mpc.Hu, 4*m_mpc.Hu );
   F.setZero();
   VectorXd F0( 2*4*m_mpc.Hu );
   F0.setZero();
   
   for ( int i = 0; i < 4*m_mpc.Hu; ++i )
   {
	  const double max_prop_speed = 1;
	  
	  E( i, i ) = 1;
	  E( 4*m_mpc.Hu + i, i ) = -1;
	  
	  F( i, i ) = 1;
	  F( 4*m_mpc.Hu + i, i ) = -1;
	  
	  // Let's say it takes 0.5 seconds to accelerate to full propeller speed?
	  E0( i ) = max_prop_speed * (TsControllerTarget() / 0.5);
	  E0( 4*m_mpc.Hu + i ) = E0(i);
	  
	  F0( i ) = max_prop_speed;
	  F0( 4*m_mpc.Hu + i ) = 0;
   }
   
   m_mpc.E = E;
   m_mpc.E0 = E0;
   
   m_mpc.F = F;
   m_mpc.F0 = F0;
}
//END class Controller

