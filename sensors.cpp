#include "sensors.h"

#include "hw/adxl345.h"
#include "hw/itg3200.h"
#include "globals.h"
#include "quad.h"

#include <cmath>
#include <iostream>
#include <unistd.h>
using namespace std;

double unifRand( double min, double max ) {
   double x = double(rand()) / RAND_MAX;
   return min + x*(max-min);
}


/**
 * Returns a value distributed as N(0,1)
 */
double normal_rv()
{
   // Approximate normal distribution as sum of 12 uniformly distributed rvs
   // in the range [-0.5, 0.5]
   
   double s = 0;
   for ( int i = 0; i < 12; ++i )
	  s += unifRand( -0.5, 0.5 );
   
   return s;
}

double normal_rv( double mu, double sigma )
{
   return normal_rv() * sigma + mu;
}

//BEGIN class CRandomWalk
CRandomWalk::CRandomWalk()
{
   m_Sigma = 0;
   m_alpha = 0;
   reset();
}

CRandomWalk::CRandomWalk( double Sigma, double alpha )
{
   m_Sigma = Sigma;
   m_alpha = alpha;
   reset();
}

void CRandomWalk::reset()
{
   m_x = 0;
   m_var = 0;
}

double CRandomWalk::step( double period, double *bias_scale, double *bias_add )
{
   if ( period == 0 )
	  return m_x;
   
   double kappa = 1.0 / (1.0 - pow( 1.0 - m_alpha*m_alpha, period ) );
   double sigma = m_Sigma / sqrt(kappa);
   
   double w = normal_rv( 0, sigma );
   
   double scale = sqrt(1.0-1.0/kappa);
   m_x = scale * m_x + w;
   m_var = scale*scale * m_var + sigma*sigma;
   
//    cout << "var = "<<scale*scale<<" * var + " << sigma*sigma << endl;
   
   if ( bias_scale )
	  *bias_scale = scale;
   if ( bias_add )
	  *bias_add = sigma*sigma;
   
   return m_x;
}
//END class CRandomWalk


//BEGIN class Sensors
Sensors::Sensors(int environment)
{
  for ( int i = 0; i < 3; ++i )
  {
    m_ba[i] = CRandomWalk( 0.2, 0.9 );
    m_bg[i] = CRandomWalk( 0.0015, 0.9 );
    m_bgps[i] = CRandomWalk( 100, 0.4 );
  }
   
  reset();

  if (environment == Globals::OnBoard) {
    accelerometer_ = new Adxl345();
    accelerometer_->initialize(true);
    gyroscope_ = new Itg3200();
    calibrate();
  } else {
    accelerometer_ = nullptr;
    gyroscope_ = nullptr;
  }
}

void Sensors::reset()
{
  m_bias_a.setZero();
  m_bias_g.setZero();
  accelerometer_bias_.setZero();
  gyro_bias_.setZero();
  
  for ( int i = 0; i < 3; ++i )
  {
  m_ba[i].reset();
  m_bg[i].reset();
  m_bgps[i].reset();
  }
}

void Sensors::calibrate() {
  cout << "Doing Sensors calibration..." << endl;
  const int discard_count = 40;
  const int measure_count = 400;
  float accel_sum_x = 0;
  float accel_sum_y = 0;
  float accel_sum_z = 0;
  float gyro_sum_x = 0;
  float gyro_sum_y = 0;
  float gyro_sum_z = 0;
  
  // Collect a second or so of data
  for (int i = 0; i < discard_count + measure_count; ++i) {
    float x, y, z;
    accelerometer_->getAcceleration(&x, &y, &z);
    if (i >= discard_count) {
      accel_sum_x += x;
      accel_sum_y += y;
      accel_sum_z += z;
    }

    gyroscope_->getRotation(&x, &y, &z);
    if (i >= discard_count) {
      gyro_sum_x += x;
      gyro_sum_y += y;
      gyro_sum_z += z;
    }

    usleep(5000); // wait for 5ms
  }

  accelerometer_bias_ = Vector3d(accel_sum_x / measure_count, accel_sum_y / measure_count, accel_sum_z / measure_count + GRAVITY);
  gyro_bias_ = Vector3d(gyro_sum_x / measure_count, gyro_sum_y / measure_count, gyro_sum_z / measure_count);
  cout << "Sensor calibration finished." << endl;
}

Vector3d stepWalk( double step, CRandomWalk *w, Vector3d x, Vector3d *bias_scale, Vector3d *bias_add )
{
   Vector3d t;
   if ( !bias_scale )
	  bias_scale = &t;
   if ( !bias_add )
	  bias_add = &t;
   for ( int i = 0; i < 3; ++i )
   {
	  w[i].step(step, &(*bias_scale)[i], &(*bias_add)[i] );
	  x(i) += w[i].value();
   }
   return x;
}

Vector3d Sensors::readAccelerometer( double step, Vector3d *bias_scale, Vector3d *bias_add )
{
  if (accelerometer_ != nullptr) {
    float x, y, z;
    accelerometer_->getAcceleration(&x, &y, &z);
    
    // TODO more generally need to do transformation of accelerometer and gyroscope to body frame.
    Vector3d adjusted(x, y, z);
    adjusted -= accelerometer_bias_;
    adjusted[2] = -adjusted[2]; // flip z-xis.
    return adjusted;
  }

  Quad *q = Globals::self().simulatedQuad();
  Vector3d a = q->info().force / q->M;
  a += GRAVITY * q->state().rotateSpaceToBody( ez );
  Vector3d x = stepWalk( step, m_ba, a, bias_scale, bias_add );
   
//    if ( World::self()->environment() == World::Simulation )
	  return x;
//    else
// 	  return World::self()->transceiver()->accel();
}

Vector3d Sensors::readGyroscope( double step, Vector3d *bias_scale, Vector3d *bias_add )
{
  if (gyroscope_ != nullptr) {
    float x, y, z;
    gyroscope_->getRotation(&x, &y, &z);
    return Vector3d(x, y, z) - gyro_bias_;
  }

  Quad *q = Globals::self().simulatedQuad();
  Vector3d g = q->state().omega;
  Vector3d x = stepWalk( step, m_bg, g, bias_scale, bias_add );
   
//    if ( World::self()->environment() == World::Simulation )
  return x;
//    else
// 	  return World::self()->transceiver()->gyro();
}

Vector3d Sensors::readGPS( double step, Vector3d *bias_scale, Vector3d *bias_add )
{
   Quad *q = Globals::self().simulatedQuad();
   Vector3d gps = q->state().pos;
   Vector3d x = stepWalk( step, m_bgps, gps, bias_scale, bias_add );
   
//    if ( World::self()->environment() == World::Simulation )
	  return x;
//    else
// 	  return World::self()->transceiver()->gps();
}

Vector3d var( CRandomWalk const *w )
{
   Vector3d m;
   for ( int i = 0; i < 3; ++i )
	  m(i) = w[i].variance();
   return m;
}

Vector3d Sensors::var_ba() const
{
   return var(m_ba);
}
Vector3d Sensors::var_bg() const
{
   return var(m_bg);
}
Vector3d Sensors::var_bgps() const
{
   return var(m_bgps);
}
//END class Sensors
