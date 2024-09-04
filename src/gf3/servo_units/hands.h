#pragma once

#include "../helpers/helpers.h"

namespace gf3 {

class Hands {
 public:
  Hands(std::string serial_port_l, std::string serial_port_r,
        const int& baudrate)
      : baudrate_{baudrate},
        serial_port_l_{serial_port_l.c_str()},
        serial_port_r_{serial_port_r.c_str()},
        fd_l_{Setup(serial_port_l_)},
        fd_r_{Setup(serial_port_r_)} {
    if (fd_l_ < 0) {
      std::cout << "Serial port setup FAILED for left hand." << std::endl;
    }
    if (fd_r_ < 0) {
      std::cout << "Serial port setup FAILED for right hand." << std::endl;
    }
  }

  ~Hands() {
    close(fd_l_);
    close(fd_r_);
  }

  int Setup(const char* serial_port) {
    int fd = open(serial_port, O_RDWR | O_NOCTTY | O_SYNC);
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

  void Send(uint8_t* suid, uint8_t* fingers) {
    // Send the start byte to left hand.
    ssize_t n = write(fd_l_, suid, 1);
    if (n < 0) {
      std::cerr << "Left hand: Error writing start byte to serial port: "
                << strerror(errno) << std::endl;
    } else if (n != 1) {
      std::cerr
          << "Left hand: Warning: Partial write of start byte to serial port"
          << std::endl;
    }

    // Send the first 5 uint8_t values to left hand.
    n = write(fd_l_, fingers, 5);
    if (n < 0) {
      std::cerr << "Left hand: Error writing hands data to serial port: "
                << strerror(errno) << std::endl;
    } else if (n != 5) {
      std::cerr
          << "Left hand: Warning: Partial write of hands data to serial port"
          << std::endl;
    }

    // Send the start byte to right hand.
    n = write(fd_r_, suid, 1);
    if (n < 0) {
      std::cerr << "Right hand: Error writing start byte to serial port: "
                << strerror(errno) << std::endl;
    } else if (n != 1) {
      std::cerr
          << "Right hand: Warning: Partial write of start byte to serial port"
          << std::endl;
    }

    // Send the last 5 uint8_t values to right hand.
    n = write(fd_r_, fingers + 5, 5);
    if (n < 0) {
      std::cerr << "Right hand: Error writing hands data to serial port: "
                << strerror(errno) << std::endl;
    } else if (n != 5) {
      std::cerr
          << "Right hand: Warning: Partial write of hands data to serial port"
          << std::endl;
    }
  }

  const int baudrate_;
  const char* serial_port_l_;
  const char* serial_port_r_;
  const int fd_l_;
  const int fd_r_;
};

}  // namespace gf3
