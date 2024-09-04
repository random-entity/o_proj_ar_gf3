#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "../servo_units/gf3.h"

namespace gf3 {

class UdpCommandReceiver {
 public:
  UdpCommandReceiver(GF3& gf3, const std::string& host, const int& port)
      : gf3_{gf3}, cfg_{.host = host, .port = port} {}

  ~UdpCommandReceiver() { close(cfg_.sock); }

 private:
  struct UdpConfig {
    const std::string host;  // Where to receive Commands from.
    const int port;
    int sock;
    sockaddr_in addr;
    socklen_t addrlen = sizeof(addr);
  } cfg_;

  union RecvBuf {
    struct Decoded {
      uint8_t suid;
      uint8_t oneshots;
      uint8_t mode;
      union {
        struct {
          uint16_t read_file_index;
          uint16_t write_file_index;
        } __attribute__((packed)) gf3;
        struct {
          float pos_out;
          float vel_out;
          float max_trq;
          float max_vel;
          float max_acc;
        } __attribute__((packed)) saj;
        struct {
          float pos_dif;
          float vel_dif;
          float pos_avg;
          float vel_avg;
          float max_trq;
          float max_vel;
          float max_acc;
        } __attribute__((packed)) dj;
        struct {
          uint8_t fingers[10];
        } __attribute__((packed)) hands;
      } u;
    } __attribute__((packed)) cmd;
    uint8_t raw_bytes[sizeof(cmd)];
  };

 public:
  bool Setup() {
    cfg_.sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (cfg_.sock < 0) {
      std::cout << "Failed to create UDP receive socket." << std::endl;
      return false;
    }
    cfg_.addr.sin_family = AF_INET;
    cfg_.addr.sin_addr.s_addr = inet_addr(cfg_.host.c_str());
    cfg_.addr.sin_port = htons(cfg_.port);
    if (bind(cfg_.sock, (struct sockaddr*)&cfg_.addr, sizeof(cfg_.addr)) < 0) {
      std::cout << "Failed to bind UDP receive socket." << std::endl;
      close(cfg_.sock);
      return false;
    }
    std::cout << "UdpCommandReceiver started listening to " << cfg_.host << ":"
              << cfg_.port << "." << std::endl;
    return true;
  }

  void Run() {
    RecvBuf rbuf;

    if (recvfrom(cfg_.sock, rbuf.raw_bytes, sizeof(rbuf.raw_bytes), 0,
                 (sockaddr*)&(cfg_.addr), &(cfg_.addrlen)) < 0) {
      std::cout << "UDP receive error!" << std::endl;
      return;
    }

    const auto suid = static_cast<int>(rbuf.cmd.suid);

    if (suid == 0) {  // GF3-level Command
      std::lock_guard lock{gf3_.cmd_.mtx};
      gf3_.cmd_.oneshots = rbuf.cmd.oneshots;
      gf3_.cmd_.read.fileindex = rbuf.cmd.u.gf3.read_file_index;
      gf3_.cmd_.write.fileindex = rbuf.cmd.u.gf3.write_file_index;
      return;
    }

    if (suid == 255) {  // Hand Command:
                        // Just forward to Pimoroni Servo 2040 hands driver.
      // Send the start byte.
      ssize_t n = write(gf3_.hands_.fd_, &rbuf.cmd.suid, sizeof(rbuf.cmd.suid));
      if (n < 0) {
        std::cerr << "Error writing start byte to serial port: "
                  << strerror(errno) << std::endl;
      } else if (n != sizeof(rbuf.cmd.suid)) {
        std::cerr << "Warning: Partial write of start byte to serial port"
                  << std::endl;
      }

      // Send the 10 uint8_t values.
      n = write(gf3_.hands_.fd_, &rbuf.cmd.u.hands, sizeof(rbuf.cmd.u.hands));
      if (n < 0) {
        std::cerr << "Error writing hands data to serial port: "
                  << strerror(errno) << std::endl;
      } else if (n != sizeof(rbuf.cmd.u.hands)) {
        std::cerr << "Warning: Partial write of hands data to serial port"
                  << std::endl;
      }

      return;
    }

    const auto maybe_saj = utils::SafeAt(gf3_.saj_map_, suid);
    if (maybe_saj) {  // SingleAxisJoint Command
      auto* j = maybe_saj.value();
      auto& cmd = j->cmd_;
      using M = SingleAxisJoint::Command::Mode;

      std::lock_guard lock{cmd.mtx};

      cmd.pos_out = std::clamp(static_cast<double>(rbuf.cmd.u.saj.pos_out),
                               j->min_pos_out_, j->max_pos_out_);
      cmd.vel_out = std::abs(static_cast<double>(rbuf.cmd.u.saj.vel_out));
      cmd.max_trq = static_cast<double>(rbuf.cmd.u.saj.max_trq);
      cmd.max_vel = static_cast<double>(rbuf.cmd.u.saj.max_vel);
      cmd.max_acc = static_cast<double>(rbuf.cmd.u.saj.max_acc);

      cmd.mode = static_cast<M>(rbuf.cmd.mode);
      switch (cmd.mode) {
        case M::Stop: {
          cmd.stop_pending = true;
        } break;
        case M::Fix: {
          cmd.fix_pending = true;
        } break;
        default:
          break;
      }

      return;
    }

    const auto maybe_dj = utils::SafeAt(gf3_.dj_map_, suid);
    if (maybe_dj) {  // DifferentialJoint Command
      auto* j = maybe_dj.value();
      auto& cmd = j->cmd_;
      using M = DifferentialJoint::Command::Mode;

      std::lock_guard lock{cmd.mtx};

      cmd.pos_dif = std::clamp(static_cast<double>(rbuf.cmd.u.dj.pos_dif),
                               j->min_pos_dif_, j->max_pos_dif_);
      cmd.vel_dif = std::abs(static_cast<double>(rbuf.cmd.u.dj.vel_dif));
      cmd.pos_avg = std::clamp(static_cast<double>(rbuf.cmd.u.dj.pos_avg),
                               j->min_pos_avg_, j->max_pos_avg_);
      cmd.vel_avg = std::abs(static_cast<double>(rbuf.cmd.u.dj.vel_avg));
      cmd.max_trq = static_cast<double>(rbuf.cmd.u.dj.max_trq);
      cmd.max_vel = static_cast<double>(rbuf.cmd.u.dj.max_vel);
      cmd.max_acc = static_cast<double>(rbuf.cmd.u.dj.max_acc);

      cmd.mode = static_cast<M>(rbuf.cmd.mode);
      switch (cmd.mode) {
        case M::Stop: {
          cmd.stop_pending = true;
        } break;
        case M::Fix: {
          cmd.fix_pending = true;
        } break;
        default:
          break;
      }

      return;
    }
  }

 private:
  GF3& gf3_;
};

}  // namespace gf3
