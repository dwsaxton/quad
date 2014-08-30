#include "pca9685.h"

#include <time.h> // TODO remove just for testing sleep
#include "i2c.h"

#include <iostream>
using namespace std;

Pca9685::Pca9685() {
  devAddr = 0x40;
  reset();
  setPWMFreq(200);
}

void Pca9685::reset(void) {
  I2c::writeByte(devAddr, PCA9685_MODE1, 0x0);
}

void Pca9685::setPWMFreq(float freq) {
  //Serial.print("Attempting to set freq ");
  //Serial.println(freq);
  freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
//   if (ENABLE_DEBUG_OUTPUT) {
//     Serial.print("Estimated pre-scale: "); Serial.println(prescaleval);
//   }
  uint8_t prescale = (int) (prescaleval + 0.5);
//   if (ENABLE_DEBUG_OUTPUT) {
//     Serial.print("Final pre-scale: "); Serial.println(prescale);
//   }
  
  uint8_t oldmode;
  I2c::readByte(devAddr, PCA9685_MODE1, &oldmode);
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  I2c::writeByte(devAddr, PCA9685_MODE1, newmode); // go to sleep
  I2c::writeByte(devAddr, PCA9685_PRESCALE, prescale); // set the prescaler
  I2c::writeByte(devAddr, PCA9685_MODE1, oldmode);
//   delay(5);
  timespec ts;
  ts.tv_sec  =  0;
  ts.tv_nsec = 5000000; // todo correct time?
  nanosleep(&ts, 0);
  I2c::writeByte(devAddr, PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
                                          // This is why the beginTransmission below was not working.
  
  uint8_t mode;
  I2c::readByte(devAddr, PCA9685_MODE1, &mode);
  cout << "Mode now: " << (int) mode << endl;
  //  Serial.print("Mode now 0x"); Serial.println(read8(PCA9685_MODE1), HEX);
}

void Pca9685::setPWM(uint8_t num, float fraction_on) {
  uint16_t on = 0;
  uint16_t off = fraction_on * 4096;
  if (off <= 0) {
    off = 1;
  } else if (off >= 4096 ) {
    off = 4095;
  }
  setPWM(num, on, off);
}

void Pca9685::setPWM(uint8_t num, uint16_t on, uint16_t off) {
  //Serial.print("Setting PWM "); Serial.print(num); Serial.print(": "); Serial.print(on); Serial.print("->"); Serial.println(off);

  I2c::writeByte(devAddr, LED0_ON_L + 4*num, on & 0xff);
  I2c::writeByte(devAddr, LED0_ON_H + 4*num, on >> 8);
  I2c::writeByte(devAddr, LED0_OFF_L + 4*num, off & 0xff);
  I2c::writeByte(devAddr, LED0_OFF_H + 4*num, off >> 8);
//   I2c::writeByte(devAddr, LED0_ON_L, on);
//   I2c::writeWord(devAddr, LED0_ON_L + 4*num, on);
//   I2c::writeWord(devAddr, LED0_ON_L + 4*num + 2, off);
//   WIRE.beginTransmission(_i2caddr);
//   WIRE.write(LED0_ON_L+4*num);
//   WIRE.write(on);
//   WIRE.write(on>>8);
//   WIRE.write(off);
//   WIRE.write(off>>8);
//   WIRE.endTransmission();
}

// Sets pin without having to deal with on/off tick placement and properly handles
// a zero value as completely off.  Optional invert parameter supports inverting
// the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
void Pca9685::setPin(uint8_t num, uint16_t val, bool invert)
{
  // Clamp value between 0 and 4095 inclusive.
  if (val > 4095) {
    val = 4095;
  }
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 4095) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, 4095-val);
    }
  }
  else {
    if (val == 4095) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 0) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, val);
    }
  }
}
