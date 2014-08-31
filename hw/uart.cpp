#include "uart.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <iostream>
#include <mutex>
using namespace std;

mutex * Uart::mutex_ = new mutex();

const int message_start_count = 4;
// random characters used as a crude hack to indicate the start of the message - framing.
// normally framing is achieved by also indicating the length of the message, so normally
// these are not needed.
char message_start[message_start_count] = {(char) 0x97, (char) 0xfa, (char) 0x46, (char) 0x13};

Uart::Uart(const char *name) {
  file_ = open(name, O_RDWR | O_NONBLOCK | O_NOCTTY);
  if (file_ < 0) {
    cerr << "Warning! failed to open uart" << endl;
  }

  struct termios tio;
  memset(&tio,0,sizeof(tio));
  tio.c_iflag=0;
  tio.c_oflag=0;
  tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
  tio.c_lflag=0;
  tio.c_cc[VMIN]=1;
  tio.c_cc[VTIME]=5;
  cfsetospeed(&tio, B57600);
  cfsetispeed(&tio, B57600);
  tcsetattr(file_, TCSANOW, &tio);
}

string Uart::getMessage() {
  lock_guard<mutex> lock(*mutex_);

  buffer_ += readData();
  // Attempt to scan buffer for a message. If a complete one found, then return it.
  for (int i = 0; i + message_start_count + 2 <= buffer_.size(); ++i) {
    bool found_magic_start = true;
    for (int j = 0; j < 4; ++j) {
      if (buffer_.at(i+j) != message_start[j]) {
        found_magic_start = false;
        break;
      }
    }
    if (!found_magic_start ) {
      continue;
    }
    // message starts at i+4;
    uint16_t length = * (uint16_t*) (buffer_.data() + i + 4);
    uint32_t start = i + 4 + 2;
    uint32_t end = start + length;
    if (buffer_.size() < end) {
      return string(); // haven't received all of the message yet
    }
    if (i > 0) {
      cerr << "Warning: discarding bytes in Uart::getMessage()" << endl;
    }
    string message = buffer_.substr(start, length);
    if (end < buffer_.length() ) {
      buffer_ = buffer_.substr(end, buffer_.length() - end);
    } else {
      buffer_.clear();
    }
    return message;
  }
  return string();
}

void Uart::writeMessage(string const& message) {
  lock_guard<mutex> lock(*mutex_);

  uint32_t length = message.size();
  if (length >= (1 << 16)) {
    cerr << "Warning! Message is longer than the maximum allowable message length. Refusing to transmit the message." << endl;
    return;
  }
  writeData(message_start, 4);
  writeData((char*) &length, 2);
  writeData(message.data(), message.size());
  cout << "Uart message written, of length " << length << endl;
}

void Uart::writeData(const char* data, int count) {
  write(file_, data, count);
}

string Uart::readData() {
  string s;
  while (true) {
    const int buffer_size = 256;
    char buffer[buffer_size];
    ssize_t count = read(file_, buffer, buffer_size);
    if (count <= 0) {
      break;
    }
    s.append(buffer, count);
    if (count < buffer_size) {
      break;
    }
  }
  return s;
}

