#pragma once

#include "../helpers/helpers.h"

namespace gf3 {

class Hands {
 public:
  Hands(std::string serial_port, const int& baudrate)
      : serial_port_{serial_port.c_str()}, baudrate_{baudrate}, fd_{Setup()} {
    if (fd_ < 0) {
      std::cout << "Serial port setup FAILED." << std::endl;
    }
  }

  ~Hands() { close(fd_); }

  int Setup() {
    int fd = open(serial_port_, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
      std::cerr << "Error opening serial port: " << strerror(errno)
                << std::endl;
      return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
      std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
      return -1;
    }

    cfsetospeed(&tty, baudrate_);
    cfsetispeed(&tty, baudrate_);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~ICRNL;

    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
      return -1;
    }

    return fd;
  }

  const char* serial_port_;
  const int baudrate_;
  const int fd_;
};

}  // namespace gf3
