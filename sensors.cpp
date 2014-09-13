#include "sensors.h"

#include "hw/adxl345.h"
#include "hw/itg3200.h"
#include "globals.h"
#include "quad.h"

#include <iostream>
using namespace std;

//BEGIN class Sensors
Sensors::Sensors(int environment)
{
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
  accelerometer_bias_.setZero();
  gyro_bias_.setZero();
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

Vector3d Sensors::readAccelerometer()
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

  // Calculate the "force" on the quadcopter in a fixed (non-inertial) reference frame.
  // Thus this includes the "force due to gravity"
  Vector3d a;
  if (Globals::self().isSimulatedQuadRunning()) {
    a = q->info().force / q->M;
  } else {
    a.setZero();
  }

  // And now make it a force in an inertial reference frame
  a += GRAVITY * q->state().rotateSpaceToBody( ez );
  return a;
}

Vector3d Sensors::readGyroscope()
{
  if (gyroscope_ != nullptr) {
    float x, y, z;
    gyroscope_->getRotation(&x, &y, &z);
    return Vector3d(x, y, z) - gyro_bias_;
  }

  Quad *q = Globals::self().simulatedQuad();
  Vector3d g = q->state().omega;
  return g;
}

Vector3d Sensors::readGPS()
{
   Quad *q = Globals::self().simulatedQuad();
   Vector3d gps = q->state().pos;
   return gps;
}
//END class Sensors
