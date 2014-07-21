#include "quad.h"

#include "world.h"

#include <cmath>
#include <QTime>

//BEGIN class QuadModel
QuadModel::QuadModel()
{
   // Default values
   M = 2;
   g = GRAVITY;
   I1 = 0.01;
   I2 = 0.01;
   I3 = 0.01;
   IP = 0.0002;
   c = Vector3d( 0, 0, -0.05 );
   d = 0.2;
   cF = 0.001;
   cT = 0.00005;
}

double QuadModel::forceFromProp( double propInput ) const
{
   double Omega = propOmega(propInput);
   return cF*Omega*Omega;
}
double QuadModel::torqueFromProp( double propInput ) const
{
   double Omega = propOmega(propInput);
   return cT*Omega*Omega;
}
double QuadModel::propOmega( double propInput ) const
{
   return 100*sqrt(propInput);
}

void QuadModel::calcDeriv( QuadState *deriv, const QuadState & state, const Vector4d & propInput, const Vector3d & windForce ) const
{
   QuadInfo info = calcInfo( state, propInput, windForce );
   
   double sumOmega = propOmega(propInput[0]) + propOmega(propInput[2]) - propOmega(propInput[1]) - propOmega(propInput[3]);
   double sumOmegaDot = 0;
   
   double w1 = state.omega.x();
   double w2 = state.omega.y();
   double w3 = state.omega.z();
   
   double iw1dot = info.torque.x() + w2*w3*(I2-I3) - IP*w2*sumOmega;
   double iw2dot = info.torque.y() + w3*w1*(I3-I1) + IP*w1*sumOmega;
   double iw3dot = info.torque.z() + w1*w2*(I1-I2) - IP*sumOmegaDot;
   
   Vector3d bodyAccel = info.force / M;
   state.calcDeriv( deriv, bodyAccel );
   deriv->omega.x() = iw1dot / I1;
   deriv->omega.y() = iw2dot / I2;
   deriv->omega.z() = iw3dot / I3;
}


QuadInfo QuadModel::calcInfo( const QuadState & state, const Vector4d & propInput, const Vector3d & windForce ) const
{
//    assert( windForce(0) + windForce(1) + windForce(2) == 0 );
   
   QuadInfo info;
   
   for ( int i = 0; i < 4; ++i )
   {
	  info.F[i] = forceFromProp( propInput[i] );
	  info.t[i] = torqueFromProp( propInput[i] );
   }
   
   // Calculate force
   info.force.setZero();
   info.force.z() += info.F[0]+info.F[1]+info.F[2]+info.F[3];
   info.force -= M * g * state.rotateSpaceToBody( ez );
   info.force += state.rotateSpaceToBody( windForce );
   
   // Calculate torque
   info.torque.setZero();
   info.torque.z() += info.t[1]+info.t[3]-info.t[0]-info.t[2];
   info.torque.y() += d*(-info.F[0]+info.F[2]);
   info.torque.x() += d*( info.F[1]-info.F[3]);
   info.torque -= (info.F[0]+info.F[1]+info.F[2]+info.F[3]) * c.cross( ez );
   
   return info;
}
//END class QuadModel


//BEGIN class QuadInfo
QuadInfo::QuadInfo()
{
   reset();
}
void QuadInfo::reset()
{
   torque.setZero();
   force.setZero();
   
   for ( int i = 0; i < 4; ++i )
   {
      t[i] = 0;
      F[i] = 0;
   }
}
//END class QuadInfo


//BEGIN class Quad
Quad::Quad()
{
   reset();
}


void Quad::reset()
{
   m_state.reset();
   m_propInput.setZero();
   m_wind.setZero();
   path_ = nullptr;
}


void Quad::setPropInput( int i, double o )
{
   m_propInput[i] = o;
}


void Quad::step()
{
   double period = TsWorldTarget();
   
   // "target" step length
   double delta = 0.01;
   
   int steps = ceil(period / delta);
   
   // correct delta
   delta = period / steps;
   
   for ( int i = 0; i < steps; ++i )
   {
      calcDeriv( &k1, m_state, m_propInput, m_wind );
      calcDeriv( &k2, m_state + k1*(delta/2), m_propInput, m_wind );
      calcDeriv( &k3, m_state + k2*(delta/2), m_propInput, m_wind );
      calcDeriv( &k4, m_state + k3*delta, m_propInput, m_wind );
      
      m_state += (k1+k2*2+k3*2+k4)*(delta/6);
      
      m_state.orient.normalize();
   }
}
//END class Quad

