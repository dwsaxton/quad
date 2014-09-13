#ifndef WORLD_H
#define WORLD_H

#include <thread>
using namespace std;

#include <Eigen/Geometry>
using namespace Eigen;

// Number of times per second the quadrocopter is numerically stepped and IMU updated
const int TsWorldMs = 20;
inline double TsWorldTarget() { return TsWorldMs*1e-3; }

// TODO should these constants be calculated?
const double MaxLinearAcceleration = 20;
const double MaxPitchAcceleration = 30;

class HigherController;

class ControlLooper : public thread {
   
public:
  ControlLooper();
  ~ControlLooper();

  bool isRunning() const { return is_running_; }
  void setRunning(bool running);

  HigherController *controller() const { return m_controller; }

private:
  void run();
  void step();
   
  HigherController *m_controller;
  bool is_running_;
  Vector4d next_input_;
  bool next_input_waiting_; // set to true when next_input_ is waiting to be sent to the propellers
};

#endif
