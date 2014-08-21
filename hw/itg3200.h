#ifndef ITG3200_H
#define ITG3200_H

class Itg3200 {
  Itg3200O(I2c *i2c, Itg3200Options options);

  /**
  * The gyroscope IC has an internal oscillator set to 1kHz (also can be set to
  * 8kHz but this class does not implement that option). The measurements of the
  * gyros are then put into the IC registers at a rate of 1000 / (\p rate + 1).
  */
  void setSampleDivider( uchar div );

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
  void read( float *x, float *y, float *z );
  
private:
  I2c *i2c_;
};

class Itg3200Options {
public:
  Options() { lowPass = lp188; sampleDiv = 0; }
  
  /**
  * Low pass frequency.
  */
  enum LowPass { lp188, lp98, lp42, lp20, lp10, lp5 };
  LowPass lowPass;
  
  /**
  * Sample rate divider. The measurement rate will be 1000 / (sampleDiv+1).
  */
  uchar sampleDiv;
};


#endif // ITG3200_H
