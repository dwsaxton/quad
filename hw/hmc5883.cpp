#include "hmc5883.h"

const uchar COMPASS_ADDR = 0x1e;

const uchar REG_CRA = 0x00; // configuration register A
const uchar REG_CRB = 0x01; // configuration register B
const uchar REG_MODE = 0x02;
const uchar REG_DATAXM = 0x03; // data output X most significant byte
// ...
const uchar REG_STATUS = 0x09; // status register

Hmc5883::Hmc5883(I2c *i2c) {
  i2c_ = i2c;

  uchar cra = 0;
  
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
  writeReg( REG_CRA, cra );
  
  // don't change CRB; thus gain is default value of +1.0 Ga, 1300 counts / milli-gauss
  
  
  // set mode to "continuous-conversion" mode (from default of sleep mode)
  writeReg( REG_MODE, 0x00 );
}

void Hmc5883::writeReg( uchar reg, uchar data )
{
   i2c_start( COMPASS_ADDR, I2C_WRITE );
   i2c_write( reg );
   i2c_write( data );
   i2c_stop();
}

void Hmc5883::readSensorReg( int16_t *x, int16_t *y, int16_t *z )
{
   i2c_start( COMPASS_ADDR, I2C_WRITE );
   i2c_write( REG_DATAXM );
   i2c_start( COMPASS_ADDR, I2C_READ );
   uchar c[6];
   for ( int i = 0; i < 6; ++i )
   {
      uchar type = (i == 5) ? I2C_READ_NACK : I2C_READ_ACK;
      i2c_read( &c[i], type );
   }
   i2c_stop();

   *x = (c[0] << 8) | c[1];
   *y = (c[2] << 8) | c[3];
   *z = (c[4] << 8) | c[5];
}
