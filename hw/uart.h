#ifndef UART_H
#define UART_H

#include <string>

namespace std {
class mutex;
};

/**
 * Uart serial communication, with framing of messages.
 */
class Uart {
public:
  Uart(const char *name);

  /**
   * Gets the next message (if there is one), or return an empty string
   */
  std::string getMessage();
  /**
   * Write a message.
   */
  void writeMessage(std::string const& message);

private:
  void writeData(const char *data, int count);
  std::string readData();

  int file_;
  std::string buffer_;
  static std::mutex* mutex_;
};

#endif // UART_H
