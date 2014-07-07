#include "quadstate.h"

//BEGIN class QuadState
QuadState::QuadState()
{
   reset();
}


void QuadState::reset()
{
   pos.setZero();
   vel.setZero();
   orient.setIdentity();
//    orient = Quaterniond( AngleAxisd( 1.4, Vector3d(0,0,1) ) );
   omega.setZero();
}

void QuadState::operator+=( const QuadState & other )
{
   pos += other.pos;
   vel += other.vel;
   orient = orient + other.orient;
   omega += other.omega;
}

void QuadState::operator-=( const QuadState & other )
{
   pos -= other.pos;
   vel -= other.vel;
   orient = orient + (-1)*other.orient;
   omega -= other.omega;
}

void QuadState::operator*=( double s )
{
   pos *= s;
   vel *= s;
   orient = s*orient;
   omega *= s;
}

QuadState QuadState::operator*( double s )
{
   QuadState qs(*this);
   qs *= s;
   return qs;
}

QuadState QuadState::operator+( const QuadState & other )
{
   QuadState qs = other;
   qs += *this;
   return qs;
}

Vector3d QuadState::rotateSpaceToBody( const Vector3d & v ) const {
   return orient * v;
}

Vector3d QuadState::rotateBodyToSpace( const Vector3d & v ) const {
   return orient.conjugate() * v;
}

Vector3d QuadState::translateSpaceToBody(Vector3d const& v) const {
  return rotateSpaceToBody(v - pos);
}

Vector3d QuadState::translateBodyToSpace(Vector3d const& v) const {
  return rotateBodyToSpace(v) + pos;
}

QuadStateVector QuadState::toVector() const
{
   QuadStateVector v;
   v.block<3,1>(0,0) = pos;
   v.block<3,1>(3,0) = vel;
   v.block<4,1>(6,0) = orient.coeffs();
   v.block<3,1>(10,0) = omega;
   return v;
}


void QuadState::fromVector( const QuadStateVector & v )
{
   pos =    v.block<3,1>(0,0);
   vel =    v.block<3,1>(3,0);
   orient = v.block<4,1>(6,0);
   omega =  v.block<3,1>(10,0);
}


void QuadState::calcDeriv( QuadState *deriv, const Vector3d & bodyAccel ) const
{
   deriv->pos = vel;
   deriv->vel = rotateBodyToSpace( bodyAccel );
   deriv->orient = derivative(orient, omega);
   deriv->omega.setZero();
}
//END class QuadState
