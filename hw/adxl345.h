#ifndef ADXL345_H
#define ADXL345_H

#include "i2c.h"

class Adxl345Options
{
public:
  Adxl345Options() {
    fifoMode = FIFO;
    accelRange = g16;
    dataRate = hz50;
  }
  
  enum FifoMode { Bypass, FIFO, Stream, Trigger };
  FifoMode fifoMode;
  
  enum AccelRange { g2, g4, g8, g16 };
  AccelRange accelRange;
  
  enum DataRate { hz3200, hz1600, hz800, hz400, hz200, hz100, hz50, hz25 };
  DataRate dataRate;
};

/*
 * Accelerometer on the 9dof stick.
 */
class Adxl345 {
public:
  Adxl345(I2c *i2c, Adxl345Options opt = Adxl345Options() );

  bool testBit( __u8 reg, __u8 bit );

  /**
  * Are there unread values on the accelerometer chip?
  */
  bool dataReady();
  /**
  * Has a data overrun occurred?
  */
  bool dataOverrun();

  __u8 readReg( __u8 reg );
  void writeReg( __u8 reg, __u8 data );
  void readSensorReg( __u16* x, __u16* y, __u16* z );
  void readAccel(float *x, float *y, float *z);

  __u8 readID();
  
private:
  I2c *i2c_;
  Adxl345Options opt_;
  float scale_;
};

#endif // ADXL345_H
