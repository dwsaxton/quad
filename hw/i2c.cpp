#include "i2c.h"

#include <errno.h>
#include <fcntl.h>
#define NULL nullptr // for i2c-dev
#include <linux/i2c-dev.h>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <iostream>
using namespace std;

int I2c::file_ = -1;
mutex * I2c::mutex_ = new mutex();

I2c::I2c() {
  if (file_ != -1) {
    cerr << "Warning: attempting to create I2c twice" << endl;
    return;
  }
  const char *filename = "/dev/i2c-1"; 
  file_ = open(filename, O_RDWR);
  if (file_ < 0) {
    std::cerr << "Failed to open the i2c bus" << std::endl;
  }
}

void print_error() {
  std::cerr << "i2c bus error: " << errno << std::endl;
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t I2c::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
  lock_guard<mutex> lock(*mutex_);
  if (ioctl(file_, I2C_SLAVE, devAddr) < 0) {
    print_error();
    return false;
  }
  int bytes_written = write(file_, &regAddr, 1);
  if (bytes_written != 1) {
    cerr << "I2c::readBytes: Requested single byte write, but " << bytes_written << " bytes actually written" << endl;
  }
  int bytes_read = read(file_, data, length);
  if (bytes_read != length) {
    cerr << "I2c::readBytes: Requested " << (int) length << " byte read, but " << bytes_read << " bytes actually read" << endl;
  }
  return bytes_read;
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2c::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
  lock_guard<mutex> lock(*mutex_);
  if (ioctl(file_, I2C_SLAVE, devAddr) < 0) {
    print_error();
    return false;
  }
  int bytes_written = write(file_, &regAddr, 1);
  if (bytes_written != 1) {
    cerr << "I2c::writeBytes: Requested single byte write, but " << bytes_written << " bytes actually written" << endl;
  }
  bytes_written = write(file_, data, length);
  if (bytes_written != length) {
    cerr << "I2c::writeBytes: Requested " << (int) length << " byte write, but " << bytes_written << " bytes actually written" << endl;
  }
  return true; // TODO error checking?
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @return Number of words read (-1 indicates failure)
 */
int8_t I2c::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data) {
  int bytesRead = readBytes(devAddr, regAddr, 2 * length, reinterpret_cast<uint8_t*>(data));
  if (bytesRead == -1) {
    return -1;
  }
  return bytesRead / 2;
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2c::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data) {
  return writeBytes(devAddr, regAddr, 2 * length, reinterpret_cast<uint8_t*>(data));
}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @return Status of read operation (true = success)
 */
int8_t I2c::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @return Status of read operation (true = success)
 */
int8_t I2c::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data) {
    uint16_t b;
    uint8_t count = readWord(devAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (true = success)
 */
int8_t I2c::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
int8_t I2c::readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data) {
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    uint8_t count;
    uint16_t w;
    if ((count = readWord(devAddr, regAddr, &w)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @return Status of read operation (true = success)
 */
int8_t I2c::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data) {
//     return readBytes(devAddr, regAddr, 1, data);
  lock_guard<mutex> lock(*mutex_);
  if (ioctl(file_, I2C_SLAVE, devAddr) < 0) {
    print_error();
  }
  __s32 result = i2c_smbus_read_byte_data(file_, regAddr);
  *data = result;
  if (result < 0) {
    print_error();
    return false;
  }
  return true;
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @return Status of read operation (true = success)
 */
int8_t I2c::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data) {
//     return readWords(devAddr, regAddr, 1, data);
  lock_guard<mutex> lock(*mutex_);
  if (ioctl(file_, I2C_SLAVE, devAddr) < 0) {
    print_error();
  }
  __s32 result = i2c_smbus_read_word_data(file_, regAddr);
  *data = result;
  if (result < 0) {
    print_error();
    return false;
  }
  return true;
}
/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2c::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2c::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
    uint16_t w;
    readWord(devAddr, regAddr, &w);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(devAddr, regAddr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2c::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devAddr, regAddr, b);
    } else {
        return false;
    }
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2c::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask byte
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    if (readWord(devAddr, regAddr, &w) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return writeWord(devAddr, regAddr, w);
    } else {
        return false;
    }
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2c::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
//   return writeBytes(devAddr, regAddr, 1, &data);
  lock_guard<mutex> lock(*mutex_);
  if (ioctl(file_, I2C_SLAVE, devAddr) < 0) {
    print_error();
  }
  __s32 res = i2c_smbus_write_byte_data(file_, regAddr, data);
  return res == 0;
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool I2c::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
//     return writeWords(devAddr, regAddr, 1, &data);
  lock_guard<mutex> lock(*mutex_);
  if (ioctl(file_, I2C_SLAVE, devAddr) < 0) {
    print_error();
  }
  __s32 res = i2c_smbus_write_word_data(file_, regAddr, data);
  return res == 0;
}