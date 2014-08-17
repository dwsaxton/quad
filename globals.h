#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>
#include <time.h>

#include <Eigen/Geometry>
using namespace Eigen;

class ControlLooper;
class Imu;
class Propellers;
class Quad;

const double GRAVITY = 9.8;
const Vector3d ez( 0, 0, 1 );

class Globals {
public:
  static Globals & self();

  ControlLooper *controlLooper() const { return control_looper_; }
  Imu *imu() const { return imu_; }
  Propellers *propellers() const { return propellers_; }
  Quad *simulatedQuad() const { return simulated_quad_; }

  /**
   * Return a monotonically increasing time from some arbitrary point, in
   * microseconds.
   */
  int64_t currentTime_us() const;

  /**
   * Sleeps the current thread for (at least) the given duration in microseconds.
   */
  void sleep_us(int64_t duration) const;
  /**
   * Sleeps the current thread until (at least) the given time.
   */
  void sleepUntil_us(int64_t time) const;

  /**
   * Resets everything (TODO does this function actually make sense?)
   */
  void reset();

private:
  void runSimulatedQuad();
  Globals();

  ControlLooper *control_looper_;
  Imu *imu_;
  Propellers *propellers_;
  Quad *simulated_quad_;

  time_t initial_seconds_;
};

#endif // GLOBALS_H
