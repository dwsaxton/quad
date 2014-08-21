#ifndef HMC5883_H
#define HMC5883_H

class I2c;

class Hmc5883 {
public:
  Hmc5883(I2c *i2c);

  /**
  * Write byte \p data to register \p reg.
  */
  void writeReg( uchar reg, uchar data );

  /**
  * Read the raw values from the sensor registers.
  */
  void readSensorReg( int16_t *x, int16_t *y, int16_t *z );

  /**
  * Get the rotation about the axes in radians per second.
  */
  // void read( float *x, float *y, float *z );
  
private:
  I2c *i2c_;
};

#endif // HMC5883_H
