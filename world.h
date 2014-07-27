#ifndef WORLD_H
#define WORLD_H

#include <Eigen/Geometry>
using namespace Eigen;

#include <QObject>

// Number of times per second the quadrocopter is numerically stepped and IMU updated
const int TsWorldMs = 12;
inline double TsWorldTarget() { return TsWorldMs*1e-3; }

// "Samping interval" Ts; the period of time each step of the controller corresponds to.
const int TsControllerMs = 4*TsWorldMs;
inline double TsControllerTarget() { return TsControllerMs*1e-3; }

const double GRAVITY = 9.8;

const int CONTROLLER_HP = 1;
const int CONTROLLER_HU = 1;

// TODO should these constants be calculated?
const double MaxLinearAcceleration = 40;
const double MaxPitchAcceleration = 30;

class QTimer;

class HigherController;
class Observer;
class Quad;
class Sensors;

double unifRand( double min, double max );

class World : public QObject
{
   Q_OBJECT
   
public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
   World();
   ~World();
   
   static World *self() { return m_self; }
   
   HigherController *controller() const { return m_controller; }
   Observer *observer() const { return m_observer; }
   Sensors *sensors() const { return m_sensors; }
   Quad *simulatedQuad() const { return m_simulatedQuad; }
   Vector3d wind() const { return m_wind; }
   void setQuadInput( Vector4d u );
   
//    double time() const { return m_stepCount * TsWorld(); }
   bool isRunning() const { return m_isRunning; }
   
   enum Environment { Simulation, Actual };
   
   void setEnvironment( Environment e );
   Environment environment() const { return m_environment; }
   
public Q_SLOTS:
   void reset();
   void runPause();
   
private Q_SLOTS:
   void step();
//    void stepIfReadings();
   
private:
   void updateTimers();
   static World *m_self;
   
   HigherController *m_controller;
   Quad *m_simulatedQuad;
   Observer *m_observer;
   Sensors *m_sensors;
//    Transceiver *m_transceiver;
   
   int m_stepCount;
   QTimer *m_stepTimer;
   QTimer *m_stepIfReadingsTimer;
   
   void stepWind();
   Vector3d m_wind;
   
   Environment m_environment;
   
   int m_sensorRequestDelay;
   bool m_isRunning;
};

#endif
