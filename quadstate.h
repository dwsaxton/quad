#ifndef QUADSTATE_H
#define QUADSTATE_H

#include <Eigen/Geometry>
using namespace Eigen;

inline Quaterniond toQuaterniond( const Vector3d & v ) {
   return Quaterniond( 0, v.x(), v.y(), v.z() );
}

inline Quaterniond operator *(double s, const Quaterniond & q ) {
   return Quaterniond( s*q.w(), s*q.x(), s*q.y(), s*q.z() );
}

inline Quaterniond operator +( const Quaterniond & p, const Quaterniond & q ) {
   return Quaterniond( p.w()+q.w(), p.x()+q.x(), p.y()+q.y(), p.z()+q.z() );
}

// return the derivative of the quaternion corresponding to the given angular velocity
inline Quaterniond derivative(const Quaterniond & original, const Vector3d & velocity) {
  return toQuaterniond(-0.5 * velocity) * original;
}
  

const int QUAD_STATE_SIZE = 13;

typedef Matrix<double, QUAD_STATE_SIZE, 1> QuadStateVector;

class QuadState
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  QuadState();
  
  // Position of body-frame-origin relative to space-frame-origin
  Vector3d pos;
  
  // Velocity of body-frame-origin relative to space-frame-origin
  Vector3d vel;
  
  // Orientation of quadrocopter
  Quaterniond orient;
  
  // Instantaneous angular velocity in the body frame
  Vector3d omega;
  
  /**
    * Calculate rate of change of quadstate with respect to time when the
    * given acceleration (in body frame) is applied.
    */
  void calcDeriv( QuadState *deriv, const Vector3d & bodyAccel ) const;
  
  QuadStateVector toVector() const;
  void fromVector( const QuadStateVector & v );
  
  void operator+=( const QuadState & other );
  void operator-=( const QuadState & other );
  QuadState operator*( double s );
  void operator*=( double s );
  QuadState operator+( const QuadState & other );
  
  // the next two functions apply just a rotation
  Vector3d rotateSpaceToBody( const Vector3d & v ) const;
  Vector3d rotateBodyToSpace( const Vector3d & v ) const;

  // the next two functions apply a rotation and a translation
  Vector3d translateSpaceToBody(Vector3d const& v) const;
  Vector3d translateBodyToSpace(Vector3d const& v) const;
  
  void reset();
  
  static int const StateIndexPos = 0;
  static int const StateIndexVel = 3;
  static int const StateIndexOrient = 6;
  static int const StateIndexomega = 10;
};

#endif
