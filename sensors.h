#ifndef SENSORS_H
#define SENSORS_H

#include <Eigen/Geometry>
using namespace Eigen;

/**
 * "Centralized" random walk: drifts back to zero.
 *  new_value = (1-1/kappa) old_value + N(0,sigma^2)
 * 
 * where  sigma  and  kappa  are chosen so that
 * 
 * variance ---> Sigma^2
 * variance-after-one-second = (alpha Sigma)^2
 */
class CRandomWalk
{
public:
   CRandomWalk();
   CRandomWalk( double Sigma, double alpha );
   void reset();
   
   /**
	* Steps with time period delta, returns next value.
	*/
   double step( double delta, double *bias_scale = 0, double *bias_add = 0 );
   
   double value() const { return m_x; }
   double variance() const { return m_var; }
   
private:
   double m_Sigma;
   double m_alpha;
   double m_x;
   double m_var;
};

class Itg3200;
class Adxl345;

class Sensors
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Sensors(int environment);
  
  void reset();
  
  /**
  * "Time step" the accelerometer by \p step. Returns the readout of the
  * accelerometer. \p bias_scale, \p bias_add are how the random walk
  * covariance scales.
  * 
  * If \p step is just equal to zero then this just returns the actual
  * acceleration plus the current bias.
  */
  Vector3d readAccelerometer( double step = 0, Vector3d *bias_scale = 0, Vector3d *bias_add = 0);
  /**
  * Similarly for gyroscope.
  */
  Vector3d readGyroscope( double step = 0, Vector3d *bias_scale = 0, Vector3d *bias_add = 0 );
  /**
  * Similarly for GPS.
  */
  Vector3d readGPS( double step = 0, Vector3d *bias_scale = 0, Vector3d *bias_add = 0 );
  /**
  * Variance of accelerometer bias.
  */
  Vector3d var_ba() const;
  /**
  * Variance of gyro bias.
  */
  Vector3d var_bg() const;
  /**
  * Variance of gps bias.
  */
  Vector3d var_bgps() const;
  
  void setBias_a( const Vector3d & b ) { m_bias_a = b; }
  void setBias_g( const Vector3d & b ) { m_bias_g = b; }
   
private:
  void calibrate();
  Vector3d accelerometer_bias_;
  Vector3d gyro_bias_;

  CRandomWalk m_ba[3];
  CRandomWalk m_bg[3];
  CRandomWalk m_bgps[3];
  
  Vector3d m_bias_a; // accelerometer zero bias
  Vector3d m_bias_g; // gyro zero bias

  // TODO make the Sensors class a Sensors interface, and have different classes to handle the on-board stuff, etc
  Itg3200 *gyroscope_;
  Adxl345 *accelerometer_;
};

#endif
