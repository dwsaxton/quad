#ifndef I2C_H
#define I2C_H

class I2c {
public:
  /**
   * Initiates i2c communication with the device at the given address (which
   * should be the 7 bit address without the read/write bit).
   */
  virtual bool startChat(uchar addr) = 0;
  /**
   * Attempt to read from the device the given number of bytes into buffer.
   * @return whether byte_count bytes were successfully read.
   */
  virtual bool read(char *buffer, int byte_count) = 0;
  /**
   * Attempt to write the given number of bytes from the buffer to the device.
   * @return whether byte_count bytes were successfully written.
   */
  virtual bool write(char *buffer, int byte_count) = 0;
};

#endif // I2C_H
