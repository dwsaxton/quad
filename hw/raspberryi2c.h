#ifndef RASPBERRYI2C_H
#define RASPBERRYI2C_H

#include "i2c.h"

class RaspberryI2c : public I2c {
public:
  RaspberryI2c();

  bool startChat(__u8 addr);
  bool read_reg(__u8 reg, __u8 *result);
  bool read_reg_2(__u8 reg, __u16 *result);
  bool write_reg(__u8 reg, __u8 value);

private:
  int file_;
};

#endif // RASPBERRYI2C_H
