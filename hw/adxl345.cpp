#include "adxl345.h"

#define ACCEL_ADDR 0x53

#define ACCEL_BW_RATE 0x2c
#define ACCEL_POWER_CTL 0x2d
#define ACCEL_INT_ENABLE 0x2e
#define ACCEL_INT_SOURCE 0x30
#define ACCEL_DATA_FORMAT 0x31
#define ACCEL_DATAX0 0x32
#define ACCEL_FIFO_CTL 0x38


void writeReg( uchar reg, uchar data )
{
   i2c_start( ACCEL_ADDR, I2C_WRITE );
   i2c_write( reg );
   i2c_write( data );
   i2c_stop();
}

Adxl345::Adxl345(I2c *i2c, Adxl345Options opt) {
// Put into standby mode to configure device
  writeReg( ACCEL_POWER_CTL, 0 );
  
  // TODO check this is indeed not necessary...
//    _delay_ms(1000);
  
  // 32 samples?
  uchar fifo = 31;
  switch ( opt.fifoMode )
  {
    case Adxl345Options::Bypass: break;
    case Adxl345Options::FIFO: fifo |= 1 << 6; break;
    case Adxl345Options::Stream: fifo |= 2 << 6; break;
    case Adxl345Options::Trigger: fifo |= 3 << 6; break;
  }
  
  writeReg( ACCEL_FIFO_CTL, fifo );
  
  uchar fmt = 0;
  switch ( opt.accelRange )
  {
    case Adxl345Options::g2: break;
    case Adxl345Options::g4: fmt |= 1; break;
    case Adxl345Options::g8: fmt |= 2; break;
    case Adxl345Options::g16: fmt |= 3; break;
  }
  
  // "Full resolution" mode
//    fmt |= 1 << 3;
  
  
  uchar rate = 0;
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
  
  writeReg( ACCEL_BW_RATE, rate );
  
  // Measurement mode
  writeReg( ACCEL_POWER_CTL, 1 << 3 );
  
//    _delay_ms(1000);
}

uchar Adxl345::readReg( uchar reg )
{
   i2c_start( ACCEL_ADDR, I2C_WRITE );
   i2c_write( reg );
   i2c_start( ACCEL_ADDR, I2C_READ );
   uchar c;
   i2c_read( &c, I2C_READ_NACK );
   i2c_stop();
   return c;
}

bool Adxl345::testBit( uchar reg, uchar bit )
{
   return readReg( reg ) & (1 << bit);
}

bool Adxl345::dataReady()
{
   return testBit( ACCEL_INT_SOURCE, 7 );
}

bool Adxl345::dataOverrun()
{
   return testBit( ACCEL_INT_SOURCE, 0 );
}

void Adxl345::readSensorReg( int16_t *x, int16_t *y, int16_t *z )
{
   i2c_start( ACCEL_ADDR, I2C_WRITE );
   i2c_write( ACCEL_DATAX0 | 0x40 | 0x80 );
   i2c_start( ACCEL_ADDR, I2C_READ );
   uchar c[6];
   for ( int i = 0; i < 6; ++i )
   {
      uchar type = (i == 5) ? I2C_READ_NACK : I2C_READ_ACK;
      i2c_read( &c[i], type );
   }
   i2c_stop();

   *x = (c[1] << 8) | c[0];
   *y = (c[3] << 8) | c[2];
   *z = (c[5] << 8) | c[4];
}

uchar Adxl345::readID()
{
   i2c_start( ACCEL_ADDR, I2C_WRITE );
   i2c_write( 0x000 );
   i2c_start( ACCEL_ADDR, I2C_READ );
   uchar c;
   i2c_read( &c, I2C_READ_NACK );
   i2c_stop();
   return c;
}
