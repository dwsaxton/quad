#include "raspberryi2c.h"

#include <cstddef>
#include <errno.h>
#include <fcntl.h>
// #include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>

#include <iostream>

RaspberryI2c::RaspberryI2c() {
  const char *filename = "/dev/i2c-2"; // TODO correct file name
  file_ = open(filename, O_RDWR);
  if (file_ < 0) {
    std::cerr << "Failed to open the i2c bus" << std::endl;
  }
}

bool RaspberryI2c::startChat(__u8 addr) {
  if (ioctl(file_, I2C_SLAVE, addr) < 0) {
    std::cerr << "Failed to acquire bus access and/or talk to slave." << std::endl;
    return false;
  }
}

void print_error() {
  std::cerr << "i2c bus error: " << errno << std::endl;
}

bool RaspberryI2c::read_reg(__u8 reg, __u8 *result) {
  __s32 reply = i2c_smbus_read_byte_data(file_, reg);
  if (reply == -1) {
    print_error();
    return false;
  }
  *result = reply;
  return true;
}

bool RaspberryI2c::read_reg_2(__u8 reg, __u16 *result) {
  __s32 reply = i2c_smbus_read_byte_data(file_, reg);
  if (reply == -1) {
    print_error();
    return false;
  }
  *result = reply;
  return true;
}
  
bool RaspberryI2c::write_reg(__u8 reg, __u8 value) {
  __s32 reply = i2c_smbus_write_byte_data(file_, reg, value);
  if (reply == -1) {
    print_error();
    return false;
  }
  return true;
}
