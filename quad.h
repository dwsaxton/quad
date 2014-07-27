#ifndef QUAD_H
#define QUAD_H

#include "linearplanner3d.h"
#include "quadstate.h"
#include "simplequadraticintercept.h"

const Vector3d ez( 0, 0, 1 ); // Vector pointing downwards

class Path;

class QuadInfo
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  QuadInfo();
  void reset();
  
  Vector3d torque;
  Vector3d force;
  
  // Individual torques and forces at each propellor
  double t[4];
  double F[4];
};

class QuadModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  QuadModel();

  double M; // mass
  double g; // gravity
  double I1, I2, I3; // inertia of body
  double IP; // inertia of propeller
  Vector3d c; // offset of center of mass
  double d;
  double cF;
  double cT;

  QuadInfo calcInfo( const QuadState & state, const Vector4d & Omega, const Vector3d & windForce = Vector3d(0,0,0) ) const;
  /**
  * windForce is another force to be added to those acting on the
  * quadrocopter, given in space frame.
  */
  void calcDeriv( QuadState * deriv, const QuadState & state, const Vector4d & Omega, const Vector3d & windForce = Vector3d(0,0,0) ) const;

  double forceFromProp( double propInput ) const;
  double torqueFromProp( double propInput ) const;
  double propOmega( double propInput ) const;
};

class Quad : public QuadModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Quad();
  
  /**
    * Resets the state to zero and the time to zero.
    */
  void reset();
  /**
    * Updates the quadrocopter position etc by stepping to the given time.
    * If period < 0, then uses the default (World) stepping time.
    */
  void step(double period = -1);
  
  /**
    * Set the input to the quadrocopter.
    */
  void setPropInput( int i, double o );
  void setPropInput( const Vector4d & o ) { m_propInput = o; }
  Vector4d propInput() const { return m_propInput; }
  void setWindForce( const Vector3d & force ) { m_wind = force; }
  
  void setState( const QuadState & s ) { m_state = s; }
  QuadState state() const { return m_state; }
  QuadInfo info() const { return calcInfo( m_state, m_propInput, m_wind ); }

  shared_ptr<Path> path_;
  LinearPlanner3d intercept;

private:
  Vector3d m_wind;
  Vector4d m_propInput;
  QuadState m_state;
  QuadState k1, k2, k3, k4;
};

#endif
