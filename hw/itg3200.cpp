#include "itg3200.h"

const uchar GYRO_ADDR = 0x68;

const uchar REG_WHO_AM_I = 0x0;
const uchar REG_SMPLRT_DIV = 0x15;
const uchar REG_DLPF_FS = 0x16;
const uchar REG_INT_CFG = 0x17;
const uchar REG_INT_STATUS = 0x1a;
const uchar REG_TEMP_OUT_H = 0x1b;
const uchar REG_TEMP_OUT_L = 0x1c;
const uchar REG_GYRO_XOUT_H = 0x1d;
// ...
const uchar REG_PWR_MGM = 0x3e;

void initDlpfReg(I2c *i2c, Options::LowPass lp )
{
  i2c_ = i2c;
  uchar dlpf = 0;
  
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
    case Options::lp188: dlpf |= 1; break;
    case Options::lp98: dlpf |= 2; break;
    case Options::lp42: dlpf |= 3; break;
    case Options::lp20: dlpf |= 4; break;
    case Options::lp10: dlpf |= 5; break;
    case Options::lp5: dlpf |= 6; break;
  }
  
  writeReg( REG_DLPF_FS, dlpf );
}

Itg3200::Itg3200O(Itg3200Options options) {
  // Reset to defaults
  writeReg( REG_PWR_MGM, 0x80 );
  
  setSampleDivider( opt.sampleDiv );
  initDlpfReg( opt.lowPass );
  
  writeReg( REG_PWR_MGM, 0x0 );
}

void Itg3200::writeReg( uchar reg, uchar data )
{
   i2c_start( GYRO_ADDR, I2C_WRITE );
   i2c_write( reg );
   i2c_write( data );
   i2c_stop();
}

void Itg3200::setSampleDivider( uchar div )
{
   writeReg( REG_SMPLRT_DIV, div );
}

void Itg3200::readSensorReg( int16_t *x, int16_t *y, int16_t *z )
{
   i2c_start( GYRO_ADDR, I2C_WRITE );
   i2c_write( REG_GYRO_XOUT_H );
   i2c_start( GYRO_ADDR, I2C_READ );
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
