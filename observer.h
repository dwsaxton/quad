#ifndef OBSERVER_H
#define OBSERVER_H

#include "quadstate.h"

const int OBSERVER_STATE_SIZE = QUAD_STATE_SIZE + 9;

typedef Matrix<double, OBSERVER_STATE_SIZE, 1> ObserverStateVector;
typedef Matrix<double, OBSERVER_STATE_SIZE, OBSERVER_STATE_SIZE> MatrixOS;

class ObserverState : public QuadState
{
public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
   ObserverState();
   ObserverState( const VectorXd & asVector );
   ObserverState( const QuadState & qs );
   
   // Acceleration bias
   Vector3d ba;
   
   // Gyro rate bias
   Vector3d bg;
   
   // GPS bias
   Vector3d bgps;
   
   ObserverStateVector toVector() const;
   void fromVector( const ObserverStateVector & v );
   
   static int const StateIndex_ba = 0;
   static int const StateIndex_bg = 3;
   static int const StateIndex_bgps = 6;
   
   void reset();
   void resetBiases();
};

class Observer
{
public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
   Observer();
   
   /**
    * Resets the state etc to zero.
    */
   void reset();
   /**
    * Updates the estimated state etc by stepping forward in time by TsNumerical.
    */
   void step();
   
   /**
	* The estimated state.
	*/
   QuadState state() const { return m_xhat; }
   /**
	* The estimated state, *before* the "update" state has been applied.
	* Thus we use accelerometer etc measurements, but not GPS.
	*/
   QuadState preUpdateState() const { return m_preUpdateState; }
   /**
	* The estimated covariance.
	*/
   MatrixOS cov() const { return m_P; }
   
private:
//    void stepSensors();
   void predict();
   void update();
   
   VectorXd m_chi0 ;
   VectorXd m_chii[2*OBSERVER_STATE_SIZE];
   VectorXd m_newchi0;
   VectorXd m_newchii[2*OBSERVER_STATE_SIZE];
   
   Vector3d m_a, m_g, m_gps;
   
   MatrixOS sqrtP();
   void genSigmaPoints();
   void propSigmaPoints();
   void estimateStateCov();
   
   ObserverState f( const ObserverState & y );
   
   QuadState integrate( QuadState state, const Vector3d & a, double t );
   
   // Estimated state
   QuadState m_xhat;
   
   QuadState m_preUpdateState;
   
   // Estimate covariance
   MatrixOS m_P;
   
   QuadState k1;
};

#endif
