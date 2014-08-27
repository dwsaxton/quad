#include "hmc5883.h"

#include "i2c.h"

const __u8 COMPASS_ADDR = 0x1e;

const __u8 REG_CRA = 0x00; // configuration register A
const __u8 REG_CRB = 0x01; // configuration register B
const __u8 REG_MODE = 0x02;
const __u8 REG_DATAXM = 0x03; // data output X most significant byte
// ...
const __u8 REG_STATUS = 0x09; // status register

Hmc5883::Hmc5883(I2c *i2c) {
  i2c_ = i2c;
  i2c_->startChat(COMPASS_ADDR);

  __u8 cra = 0;
  
  // minimum data output rate (Hz)
  // 0  0.75
  // 1  1.5
  // 2  3
  // 3  7.5
  // 4  15
  // 5  30
  // 6  75
  // 7  not used
  
  // 75 Hz
  cra |= 6 << 2;
  i2c_->write_reg(REG_CRA, cra);
  
  // don't change CRB; thus gain is default value of +1.0 Ga, 1300 counts / milli-gauss
  
  // set mode to "continuous-conversion" mode (from default of sleep mode)
  i2c_->write_reg(REG_MODE, 0x00);
}

void Hmc5883::readSensorReg( __u16 *x, __u16 *y, __u16 *z )
{
  i2c_->startChat(COMPASS_ADDR);
  i2c_->read_reg_2(REG_DATAXM + 0, x);
  i2c_->read_reg_2(REG_DATAXM + 2, y);
  i2c_->read_reg_2(REG_DATAXM + 4, z);
}
