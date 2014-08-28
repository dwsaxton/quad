#include "adxl345.h"

#include <stdint.h>

#include "i2c.h"

#define ACCEL_ADDR 0x53

#define ACCEL_BW_RATE 0x2c
#define ACCEL_POWER_CTL 0x2d
#define ACCEL_INT_ENABLE 0x2e
#define ACCEL_INT_SOURCE 0x30
#define ACCEL_DATA_FORMAT 0x31
#define ACCEL_DATAX0 0x32
#define ACCEL_FIFO_CTL 0x38

Adxl345::Adxl345(I2c *i2c, Adxl345Options opt) {
  opt_ = opt;
  i2c_ = i2c;
  i2c_->startChat(ACCEL_ADDR);
// Put into standby mode to configure device
  i2c_->write_reg(ACCEL_POWER_CTL, 0);
  
  // TODO check this is indeed not necessary...
//    _delay_ms(1000);
  
  // 32 samples?
  __u8 fifo = 31;
  switch ( opt.fifoMode )
  {
    case Adxl345Options::Bypass: break;
    case Adxl345Options::FIFO: fifo |= 1 << 6; break;
    case Adxl345Options::Stream: fifo |= 2 << 6; break;
    case Adxl345Options::Trigger: fifo |= 3 << 6; break;
  }
  
  i2c_->write_reg(ACCEL_FIFO_CTL, fifo);
  
  __u8 fmt = 0;
  scale_ = 2 * 9.81 / (1 << 10);
  switch ( opt.accelRange )
  {
    case Adxl345Options::g2:
      scale_ *= 2;
      break;
    case Adxl345Options::g4:
      scale_ *= 4;
      fmt |= 1;
      break;
    case Adxl345Options::g8:
      scale_ *= 8;
      fmt |= 2;
      break;
    case Adxl345Options::g16:
      scale_ *= 16;
      fmt |= 3;
      break;
  }
  
  // "Full resolution" mode
//    fmt |= 1 << 3;
  
  __u8 rate = 0;
  switch ( opt.dataRate )
  {
    case Adxl345Options::hz3200: rate |= 0b1111; break;
    case Adxl345Options::hz1600: rate |= 0b1110; break;
    case Adxl345Options::hz800:  rate |= 0b1101; break;
    case Adxl345Options::hz400:  rate |= 0b1100; break;
    case Adxl345Options::hz200:  rate |= 0b1011; break;
    case Adxl345Options::hz100:  rate |= 0b1010; break;
    case Adxl345Options::hz50:   rate |= 0b1001; break;
    case Adxl345Options::hz25:   rate |= 0b1000; break;
  }
  
  i2c_->write_reg(ACCEL_BW_RATE, rate);
  
  // Measurement mode
  i2c_->write_reg(ACCEL_POWER_CTL, 1 << 3);
  
//    _delay_ms(1000);
}

bool Adxl345::testBit( __u8 reg, __u8 bit ) {
  i2c_->startChat(ACCEL_ADDR);
  __u8 c;
  i2c_->read_reg(reg, &c);
   return c & (1 << bit);
}

bool Adxl345::dataReady()
{
  return testBit(ACCEL_INT_SOURCE, 7);
}

bool Adxl345::dataOverrun()
{
  return testBit(ACCEL_INT_SOURCE, 0);
}

void Adxl345::readSensorReg(__u16 *x, __u16 *y, __u16 *z)
{
  i2c_->startChat(ACCEL_ADDR);
  i2c_->read_reg_2((ACCEL_DATAX0 | 0x40 | 0x80) + 0, x);
  i2c_->read_reg_2((ACCEL_DATAX0 | 0x40 | 0x80) + 2, y);
  i2c_->read_reg_2((ACCEL_DATAX0 | 0x40 | 0x80) + 4, z);
}

void Adxl345::readAccel(float *x, float *y, float *z) {
  __u16 x_, y_, z_;
  readSensorReg(&x_, &y_, &z_);
  *x = scale_ * *reinterpret_cast<int16_t*>(&x_);
  *y = scale_ * *reinterpret_cast<int16_t*>(&y_);
  *z = scale_ * *reinterpret_cast<int16_t*>(&z_);
}

__u8 Adxl345::readID() {
  i2c_->startChat(ACCEL_ADDR);
  __u8 c;
  i2c_->read_reg(0x00, &c);
  return c;
}
