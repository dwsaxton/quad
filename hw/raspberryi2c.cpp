#include "raspberryi2c.h"

#include <errno.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>

#include <iostream>

RaspberryI2c::RaspberryI2c() {
  char *filename = "/dev/i2c-2"; // TODO correct file name
  file_ = open(filename, O_RDWR);
  if (file_ < 0) {
    std::cerr << "Failed to open the i2c bus" << std::endl;
  }
}

bool RaspberryI2c::startChat(uchar addr) {
  if (ioctl(file_, I2C_SLAVE, addr) < 0) {
    std::cerr << "Failed to acquire bus access and/or talk to slave." << std::endl;
    return false;
  }
}

bool RaspberryI2c::read(char *buffer, int byte_count) {
  if (read(file_, buffer, byte_count) != byte_count) {
    std::cerr << "Failed to read from the i2c bus: " << errno << std::endl;
    return false;
  }
  return true;
}

bool RaspberryI2c::write(char *buffer, int byte_count) {
  if (::write(file_, buffer, byte_count) != byte_count) {
    std::cerr << "Failed to read from the i2c bus: " << errno << std::endl;
    return false;
  }
  return true;
}
