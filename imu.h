#ifndef IMU_H
#define IMU_H

#include <Eigen/Geometry>
using namespace Eigen;

class I2c;
class MemsImu;
class QuadState;

class Imu {
public:
  Imu(int environment, I2c* i2c);
  ~Imu();

  /**
   * Return a best guess for the quad state at the given time, which should be
   * in the future (possibly works if in past as well, as long as not too far).
   * This function  is implemented with the expectation that time is at most
   * tens of  milliseconds into the future, since it uses the assumption that
   * the current linear and angular acceleration will not change from the last
   * point they were measured to the given time.)
   */
  QuadState stateForTime(int64_t time) const;

  /**
   * The last sensor acceleration reading (possibly corrected for bias - not yet
   * implemented at the time of writing this comment).
   */
  Vector3d lastAcceleration() const;
  /**
   * Similar as above, but for angular acceleration.
   */
  Vector3d lastAngularAcceleration() const;

  void reset();

private:
  MemsImu *mems_imu_;
};

#endif // IMU_H
