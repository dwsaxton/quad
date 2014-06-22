#ifndef QUADSTATE_H
#define QUADSTATE_H

#include <Eigen/Geometry>
using namespace Eigen;

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
      
      Vector3d spaceToBody( const Vector3d & v ) const;
      Vector3d bodyToSpace( const Vector3d & v ) const;
      
      void reset();
      
      static int const StateIndexPos = 0;
      static int const StateIndexVel = 3;
	  static int const StateIndexOrient = 6;
	  static int const StateIndexomega = 10;
};

#endif
