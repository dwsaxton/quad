#include "world.h"

#include "controller.h"
#include "observer.h"
#include "quad.h"
#include "sensors.h"

#include <QTimer>
#include <cassert>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <sys/time.h>
using namespace std;

World* World::m_self = 0;

// Number of times per second the quadrocopter is numerically stepped and IMU updated
const int TsWorldMs = 12;
double TsWorldTarget() { return TsWorldMs*1e-3; }

// "Samping interval" Ts; the period of time each step of the controller corresponds to.
const int TsControllerMs = 4*TsWorldMs;
double TsControllerTarget() { return TsControllerMs*1e-3; }

World::World()
{
   assert( !m_self );
   m_self = this;
   
   // Want sampling interval to be an integer multiple of the quad step
   assert( TsControllerMs % TsWorldMs == 0 );
   
   m_wind.setZero();
   
   m_simulatedQuad = new Quad;
   m_controller = new Controller;
   m_observer = new Observer;
   m_sensors = new Sensors;
//    m_transceiver = new Transceiver;
   
   m_isRunning = false;
   m_sensorRequestDelay = 0;
   m_lastControllerTime = 0;
   m_lastWorldTime = 0;
   m_tsControllerActual = 0;
   m_tsWorldActual = 0;
   m_environment = Simulation;
   m_stepCount = 0;
   m_stepTimer = new QTimer(this);
   m_stepIfReadingsTimer = new QTimer(this);
   connect( m_stepTimer, SIGNAL(timeout()), this, SLOT(step()) );
   connect( m_stepIfReadingsTimer, SIGNAL(timeout()), this, SLOT(stepIfReadings()) );
}

World::~World()
{
   delete m_controller;
   delete m_simulatedQuad;
   delete m_observer;
   delete m_sensors;
}


void World::setEnvironment( Environment e )
{
   if ( m_environment == e )
	  return;
   m_environment = e;
   
   if ( e == Simulation )
   {
//       m_transceiver->end();
	  // Switching to simulation mode, let's update the internal quad state
	  // to that predicted of the actual quad state
	  m_simulatedQuad->setState( m_observer->state() );
   }
   else
   {
//       m_transceiver->start();
   }
   
   updateTimers();
}

// void World::stepIfReadings()
// {
//    if ( m_transceiver->trySensorReadings() )
//       step();
// }

void World::step()
{
   bool stepController = ( m_stepCount % (TsControllerMs / TsWorldMs) == 0 );
   
   // Controller
   if ( m_controller->haveControl() && stepController )
   {
	  m_controller->step();
   }
   
   if ( m_environment == Simulation )
   {
	  stepWind();
	  m_simulatedQuad->step();
   }
   else
   {
      // Commented out: if this function is called then already have data waiting
//       m_transceiver->readData();
   }
   
   m_observer->step();
   
   m_stepCount++;
   
   // Reset timers
   getWorldTime();
   if ( stepController )
      getControllerTime();
}

double unifRand( double min, double max )
{
   double x = double(rand()) / RAND_MAX;
   return min + x*(max-min);
}

void World::stepWind()
{
   // No wind for now
//    return;
   
   double maxWind = 1; // max number of newtons of force
   
   // Average time of change
   double rate = 0.05 / maxWind;
   
   for ( int i = 0; i < 3; ++i )
   {
	  double x = m_wind(i);
	  
	  double maxDelta = TsWorldTarget() / rate;
	  
	  x += unifRand( -maxDelta, maxDelta );
	  
	  if ( x > maxWind )
		 x = maxWind;
	  if ( x < -maxWind )
		 x = -maxWind;
	  
	  m_wind(i) = x;
   }
   
   m_simulatedQuad->setWindForce(m_wind);
}


void World::reset()
{
   m_stepTimer->stop();
   m_simulatedQuad->reset();
   m_sensors->reset();
   m_observer->reset();
   m_stepCount = 0;
}

void World::runPause()
{
   m_isRunning = !m_isRunning;
//    if ( isRunning() )
//       m_stepTimer->stop();
//    else
   if (m_isRunning)
   {
      getControllerTime();
      getWorldTime();
//       m_stepTimer->start( 1000 / WorldStepsPerSecond );
      
//       if ( m_environment == Actual )
//          QTimer::singleShot( 1000 / WorldStepsPerSecond - 10, m_transceiver, SLOT(sendSensorRequest()) );
   }
   updateTimers();
}

void World::updateTimers()
{
   m_stepTimer->stop();
   m_stepIfReadingsTimer->stop();
   
   if ( isRunning() )
   {
      if ( m_environment == Simulation )
         m_stepTimer->start( TsWorldMs );
      else
         m_stepIfReadingsTimer->start(1);
   }
}

void World::setQuadInput( Vector4d u )
{
   if ( m_environment == Simulation )
	  m_simulatedQuad->setPropInput( u );
   else
   {} // TODO handle this
}

double currentTime()
{
   struct timeval t;
   gettimeofday( &t, 0 );
   int sec = t.tv_sec - 1403300000;
   return sec + t.tv_usec*1e-6;
}

void World::getWorldTime()
{
   double t = currentTime();
   m_tsWorldActual = t - m_lastWorldTime;
   if ( m_tsWorldActual < 0 || m_tsWorldActual > 1 )
   {
      cout << "Warning: dodgy world time" << endl;
      m_tsWorldActual = TsWorldTarget();
   }
   m_lastWorldTime = t;
}
void World::getControllerTime()
{
   double t = currentTime();
   m_tsControllerActual = t - m_lastControllerTime;
   if ( m_tsControllerActual < 0 || m_tsControllerActual > 1 )
   {
      cout << "Warning: dodgy controller time" << endl;
      m_tsControllerActual = TsControllerTarget();
   }
   m_lastControllerTime = t;
}
