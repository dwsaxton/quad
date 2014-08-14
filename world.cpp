#include "world.h"

#include "highercontroller.h"
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

World *World::self() {
  if (!m_self) {
    m_self = new World();
  }
  return m_self;
}

World::World() {
  assert( !m_self );
  m_self = this;
  
  m_wind.setZero();
  
  m_simulatedQuad = new Quad;
  m_controller = new HigherController;
  m_observer = new Observer;
  m_sensors = new Sensors;
  
  m_haveAutomaticControl = true;
  m_isRunning = false;
  m_stepCount = 0;
  m_stepTimer = new QTimer(this);
  m_stepIfReadingsTimer = new QTimer(this);
  connect( m_stepTimer, SIGNAL(timeout()), this, SLOT(step()) );
  connect( m_stepIfReadingsTimer, SIGNAL(timeout()), this, SLOT(stepIfReadings()) );
}

World::~World() {
   delete m_controller;
   delete m_simulatedQuad;
   delete m_observer;
   delete m_sensors;
}

void World::step() {
  if (m_haveAutomaticControl) {
    Vector4d new_input = m_controller->getPropInputs(m_simulatedQuad);
    m_simulatedQuad->setPropInput(new_input); // TODO do at correct time
  }

  stepWind();
  m_simulatedQuad->step();

//   m_observer->step();
  m_stepCount++;
}

double unifRand( double min, double max ) {
   double x = double(rand()) / RAND_MAX;
   return min + x*(max-min);
}

void World::stepWind() {
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

void World::reset() {
  m_stepTimer->stop();
  m_simulatedQuad->reset();
  m_sensors->reset();
  m_observer->reset();
  m_stepCount = 0;
}

void World::runPause() {
  m_isRunning = !m_isRunning;
  updateTimers();
}

void World::updateTimers() {
  m_stepTimer->stop();
  m_stepIfReadingsTimer->stop();
  
  if (isRunning()) {
    m_stepTimer->start(1 * TsWorldMs);
  }
}

void World::setManualQuadInput( Vector4d u ) {
  m_simulatedQuad->setPropInput( u );
}

void World::setAutomaticControl(bool automatic) {
  m_haveAutomaticControl = automatic;
}
