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
  file_ = open(name, O_RDWR | O_NONBLOCK);  

//   struct termios old_stdio;
//   tcgetattr(STDOUT_FILENO, &old_stdio);

  struct termios stdio;
  memset(&stdio, 0, sizeof(stdio));
  stdio.c_iflag=0;
  stdio.c_oflag=0;
  stdio.c_cflag=0;
  stdio.c_lflag=0;
  stdio.c_cc[VMIN]=1;
  stdio.c_cc[VTIME]=0;
  tcsetattr(STDOUT_FILENO, TCSANOW, &stdio);
  tcsetattr(STDOUT_FILENO, TCSAFLUSH, &stdio);
  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking

  struct termios tio;
  memset(&tio,0,sizeof(tio));
  tio.c_iflag=0;
  tio.c_oflag=0;
  tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
  tio.c_lflag=0;
  tio.c_cc[VMIN]=1;
  tio.c_cc[VTIME]=5;
  cfsetospeed(&tio, B115200);            // 115200 baud
  cfsetispeed(&tio, B115200);            // 115200 baud
  tcsetattr(file_, TCSANOW, &tio);

//   char c = 'D';
//   while (c!='q')
//   {
//     if (read(file_, &c, 1)>0) {
//       write(STDOUT_FILENO, &c, 1);              // if new data is available on the serial port, print it out
//     }
//     if (read(STDIN_FILENO, &c, 1) > 0) {
//       write(file_, &c, 1);                     // if new data is available on the console, send it to the serial port
//     }
//   }

//   close(file_);
//   tcsetattr(STDOUT_FILENO,TCSANOW,&old_stdio);
//   return EXIT_SUCCESS;
}

string Uart::getMessage() {
  lock_guard<mutex> lock(*mutex_);

  buffer_ += readData();
  // Attempt to scan buffer for a message. If a complete one found, then return it.
  for (int i = 0; i + message_start_count + 2 <= buffer_.size(); ++i) {
    for (int j = 0; j < 4; ++j) {
      if (buffer_.at(i+j) != message_start[j]) {
        continue;
      }
    }
    // message starts at i+4;
    uint16_t length = * (uint16_t*) (buffer_.data() + i + 4);
    uint32_t start = i + 4 + 2;
    uint32_t end = start + length;
    if (buffer_.size() < end) {
      return string(); // haven't received all of the message yet
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

