#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>
#include <string>
#include <time.h>

#include <Eigen/Geometry>
using namespace Eigen;

class ControlLooper;
class I2c;
class Imu;
class Propellers;
class Quad;
class Uart;

const double GRAVITY = 9.8;
const Vector3d ez( 0, 0, 1 );

class Globals {
public:
  static Globals & self();

  ControlLooper *controlLooper() const { return control_looper_; }
  Imu *imu() const { return imu_; }
  Propellers *propellers() const { return propellers_; }
  /**
   * When we are in a simulation environment (running on a laptop), this is the
   * simulated quad.
   */
  Quad *simulatedQuad() const { return simulated_quad_; }
  /**
   * When we are running on a laptop in Receiving environment, this is the
   * state of the remote quad.
   */
  Quad *remoteQuad() const { return remote_quad_; }
  I2c *i2c() const { return i2c_; }
  Uart *uart() const { return uart_; }
  void setSimulatedQuadRunning(bool run) { simulated_quad_running_ = run; }

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

  enum Environment {
    OnBoard,
    Simulation,
    Receiving,
  };

  /**
   * Obtain the environment we're running in (whether we're onboard the quad,
   * or in a simulation environment, or receiving quad information.
   */
  Environment environment() const { return environment_; }

private:
  Globals();
  void runSimulatedQuad();
  /**
   * Send and receive messages from the other device (be it the laptop or the
   * quad depending on which device this is).
   */
  void doCommunicationLoop();
  /**
   * Interpret a received message.
   */
  void handleMessage(std::string const& message);

  ControlLooper *control_looper_;
  Imu *imu_;
  Propellers *propellers_;
  Quad *simulated_quad_;
  Quad *remote_quad_;
  I2c *i2c_;
  Environment environment_;
  Uart *uart_;

  time_t initial_seconds_;
  bool simulated_quad_running_;
};

#endif // GLOBALS_H
