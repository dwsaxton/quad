#ifndef HMC5883_H
#define HMC5883_H

#include "i2c.h"

/**
 * Compass on 9dof stick.
 */
class Hmc5883 {
public:
  Hmc5883(I2c *i2c);

  /**
  * Read the raw values from the sensor registers.
  */
  void readSensorReg(__u16 *x, __u16 *y, __u16 *z);

  /**
  * Get the rotation about the axes in radians per second.
  */
  // void read( float *x, float *y, float *z );
  
private:
  I2c *i2c_;
};

#endif // HMC5883_H
