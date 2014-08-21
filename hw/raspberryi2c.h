#ifndef RASPBERRYI2C_H
#define RASPBERRYI2C_H

#include "i2c.h"

class RaspberryI2c : public I2c {
public:
  RaspberryI2c();

  bool startChat(uchar addr);
  bool read(char *buffer, int byte_count);
  bool write(char *buffer, int byte_count);

private:
  int file_;
};

#endif // RASPBERRYI2C_H
