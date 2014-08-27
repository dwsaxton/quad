#ifndef ITG3200_H
#define ITG3200_H

#include "i2c.h"

class Itg3200Options {
public:
  Itg3200Options() { lowPass = lp188; sampleDiv = 0; }
  
  /**
  * Low pass frequency.
  */
  enum LowPass { lp188, lp98, lp42, lp20, lp10, lp5 };
  LowPass lowPass;
  
  /**
  * Sample rate divider. The measurement rate will be 1000 / (sampleDiv+1).
  */
  __u8 sampleDiv;
};

class Itg3200 {
  Itg3200(I2c *i2c, Itg3200Options options);

  /**
  * The gyroscope IC has an internal oscillator set to 1kHz (also can be set to
  * 8kHz but this class does not implement that option). The measurements of the
  * gyros are then put into the IC registers at a rate of 1000 / (\p rate + 1).
  */
  void setSampleDivider( __u8 div );

  /**
  * Read the raw values from the sensor registers.
  */
  void readSensorReg( __u16 *x, __u16 *y, __u16 *z );

  /**
  * Get the rotation about the axes in radians per second.
  */
  void read( float *x, float *y, float *z );
  
private:
  void initDlpfReg(Itg3200Options::LowPass lp);

  I2c *i2c_;
};

#endif // ITG3200_H
