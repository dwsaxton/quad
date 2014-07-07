#include "sensors.h"

#include "quad.h"
#include "world.h"

#include <cmath>
#include <iostream>
using namespace std;

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
Sensors::Sensors()
{
   for ( int i = 0; i < 3; ++i )
   {
	  m_ba[i] = CRandomWalk( 0.2, 0.9 );
      m_bg[i] = CRandomWalk( 0.0015, 0.9 );
	  m_bgps[i] = CRandomWalk( 100, 0.4 );
   }
   
   reset();
}

void Sensors::reset()
{
   m_bias_a.setZero();
   m_bias_g.setZero();
   
   for ( int i = 0; i < 3; ++i )
   {
	  m_ba[i].reset();
	  m_bg[i].reset();
	  m_bgps[i].reset();
   }
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
   Quad *q = World::self()->simulatedQuad();
   Vector3d a = q->info().force / q->M;
   a += 9.81 * q->state().rotateSpaceToBody( ez );
   Vector3d x = stepWalk( step, m_ba, a, bias_scale, bias_add );
   
//    if ( World::self()->environment() == World::Simulation )
	  return x;
//    else
// 	  return World::self()->transceiver()->accel();
}

Vector3d Sensors::readGyroscope( double step, Vector3d *bias_scale, Vector3d *bias_add )
{
   Quad *q = World::self()->simulatedQuad();
   Vector3d g = q->state().omega;
   Vector3d x = stepWalk( step, m_bg, g, bias_scale, bias_add );
   
//    if ( World::self()->environment() == World::Simulation )
	  return x;
//    else
// 	  return World::self()->transceiver()->gyro();
}

Vector3d Sensors::readGPS( double step, Vector3d *bias_scale, Vector3d *bias_add )
{
   Quad *q = World::self()->simulatedQuad();
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
