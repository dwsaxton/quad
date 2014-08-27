#ifndef I2C_H
#define I2C_H

#include <linux/types.h>

class I2c {
public:
  /**
   * Initiates i2c communication with the device at the given address (which
   * should be the 7 bit address without the read/write bit).
   */
  virtual bool startChat(__u8 addr) = 0;
  /**
   * Attempt to read a byte from the register on the device.
   * @return whether the read was successful.
   */
  virtual bool read_reg(__u8 reg, __u8 *result) = 0;
  /**
   * Attempt to read two bytes from the register on the device.
   * @return whether the read was successful.
   */
  virtual bool read_reg_2(__u8 reg, __u16 *result) = 0;
  /**
   * Attempt to write a byte to the given register on the byte.
   * @return whether the write was successful.
   */
  virtual bool write_reg(__u8 reg, __u8 value) = 0;

};

#endif // I2C_H
