#ifndef WORLD_H
#define WORLD_H

#include <Eigen/Geometry>
using namespace Eigen;

#include <QObject>

// "Samping interval" Ts; the period of time each step of the controller corresponds to.
double TsControllerTarget();

// What is the time step of the world? How often the quadrocopter is numerically stepped, the IMU updated, etc.
double TsWorldTarget();

class QTimer;

class Controller;
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
   
   Controller *controller() const { return m_controller; }
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
   
   // The actually number of elapsed seconds since the last time the controller step function was called
   ///TODO Is the following function actually used? We could potentially get rid of it...
   double TsControllerActual() const { return m_tsControllerActual; }
   double TsWorldActual() const { return m_tsWorldActual; }
   
public Q_SLOTS:
   void reset();
   void runPause();
   
private Q_SLOTS:
   void step();
//    void stepIfReadings();
   
private:
   void updateTimers();
   static World *m_self;
   
   Controller *m_controller;
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
   
   /// Updates the controller time to the current time
   void getControllerTime();
   /// Updates the world time to the current time
   void getWorldTime();
   double m_lastControllerTime;
   double m_lastWorldTime;
   double m_tsControllerActual;
   double m_tsWorldActual;
   int m_sensorRequestDelay;
   bool m_isRunning;
};

#endif
