#include "itg3200.h"

#include <cmath>
#include <cstdint>

#include "i2c.h"

const __u8 GYRO_ADDR = 0x68;

const __u8 REG_WHO_AM_I = 0x0;
const __u8 REG_SMPLRT_DIV = 0x15;
const __u8 REG_DLPF_FS = 0x16;
const __u8 REG_INT_CFG = 0x17;
const __u8 REG_INT_STATUS = 0x1a;
const __u8 REG_TEMP_OUT_H = 0x1b;
const __u8 REG_TEMP_OUT_L = 0x1c;
const __u8 REG_GYRO_XOUT_H = 0x1d;
// ...
const __u8 REG_PWR_MGM = 0x3e;

Itg3200::Itg3200(I2c *i2c, Itg3200Options options) {
  i2c_ = i2c;
  i2c_->startChat(GYRO_ADDR);
  // Reset to defaults
  i2c_->write_reg(REG_PWR_MGM, 0x80);
  
  setSampleDivider(options.sampleDiv);
  initDlpfReg(options.lowPass );
  
  i2c_->write_reg( REG_PWR_MGM, 0x0 );
}

void Itg3200::initDlpfReg(Itg3200Options::LowPass lp) {
  char dlpf = 0;
  
  // Required for proper gyro operation, sets full-scale range of gyros
  dlpf |= 3 << 3;
  
  // First three bits of dlpf select the internal sample rate and low pass
  // filter bandwidth, as detailed below.
  // 
  // 0 256Hz 8kHz
  // 1 188Hz 1kHz
  // 2 98Hz 1kHz
  // 3 42Hz 1kHz
  // 4 20Hz 1kHz
  // 5 10Hz 1kHz
  // 6 5Hz 1kHz
  // 7 reserved
  
  // Low pass filter of 188Hz
  switch ( lp )
  {
    case Itg3200Options::lp188: dlpf |= 1; break;
    case Itg3200Options::lp98: dlpf |= 2; break;
    case Itg3200Options::lp42: dlpf |= 3; break;
    case Itg3200Options::lp20: dlpf |= 4; break;
    case Itg3200Options::lp10: dlpf |= 5; break;
    case Itg3200Options::lp5: dlpf |= 6; break;
  }

  i2c_->startChat(GYRO_ADDR);
  i2c_->write_reg(REG_DLPF_FS, dlpf);
}

void Itg3200::setSampleDivider( __u8 div )
{
  i2c_->startChat(GYRO_ADDR);
  i2c_->write_reg( REG_SMPLRT_DIV, div );
}

void Itg3200::readSensorReg( __u16 *x, __u16 *y, __u16 *z ) {
  i2c_->startChat(GYRO_ADDR);
  i2c_->read_reg_2(REG_GYRO_XOUT_H + 0, x);
  i2c_->read_reg_2(REG_GYRO_XOUT_H + 2, z);
  i2c_->read_reg_2(REG_GYRO_XOUT_H + 4, x);
}

void Itg3200::read( float *x, float *y, float *z ) {
  __u16 x_, y_, z_;
  readSensorReg(&x_, &y_, &z_);
  float scale = (M_PI / 180.0) * (1.0 / 14.375); // as per the datasheet
  *x = scale * *reinterpret_cast<int16_t*>(&x_);
  *y = scale * *reinterpret_cast<int16_t*>(&y_);
  *z = scale * *reinterpret_cast<int16_t*>(&z_);
}

