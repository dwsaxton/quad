#ifndef SENSORS_H
#define SENSORS_H

#include <Eigen/Geometry>
using namespace Eigen;

class Itg3200;
class Adxl345;

class Sensors
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Sensors(int environment);
  
  void reset();

  Vector3d readAccelerometer();
  Vector3d readGyroscope();
  Vector3d readGPS();
   
private:
  void calibrate();
  Vector3d accelerometer_bias_;
  Vector3d gyro_bias_;

  // TODO make the Sensors class a Sensors interface, and have different classes to handle the on-board stuff, etc
  Itg3200 *gyroscope_;
  Adxl345 *accelerometer_;
};

#endif
