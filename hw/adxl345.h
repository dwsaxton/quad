#ifndef ADXL345_H
#define ADXL345_H

/*
 * Accelerometer on the 9dof stick.
 */
class Adxl345 {
public:
  void Adxl345(I2c *i2c, Adxl345Options opt = Adxl345Options() );

  bool testBit( uchar reg, uchar bit );

  /**
  * Are there unread values on the accelerometer chip?
  */
  bool dataReady();
  /**
  * Has a data overrun occurred?
  */
  bool dataOverrun();

  uchar readReg( uchar reg );
  void writeReg( uchar reg, uchar data );
  void readSensorReg( int16_t *x, int16_t *y, int16_t *z );

  uchar readID();
  
private:
  I2c *i2c_;
};


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

#endif // ADXL345_H
