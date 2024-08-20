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

  struct UdpConfig {
    const std::string host;  // Where to receive Commands from.
    const int port;
    int sock;
    sockaddr_in addr;
    socklen_t addrlen = sizeof(addr);
  } cfg_;

  union RecvBuf {
    struct Decoded {
      uint8_t id;
      uint8_t mode;
      union {
        struct {
          float target_out;
          float vel;
          float max_trq, max_vel, max_acc;
        } __attribute__((packed)) saj;
        struct {
          float target_avg, target_dif;
          float vel;
          float max_trq, max_vel, max_acc;
        } __attribute__((packed)) dj;
      } u;
    } __attribute__((packed)) cmd;
    uint8_t raw_bytes[sizeof(cmd)];
  };

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
    return true;
  }

  void Run() {
    RecvBuf rbuf;

    if (recvfrom(cfg_.sock, rbuf.raw_bytes, sizeof(rbuf.raw_bytes), 0,
                 (sockaddr*)&(cfg_.addr), &(cfg_.addrlen)) < 0) {
      std::cout << "UDP receive error!" << std::endl;
      return;
    }

    const auto id = static_cast<int>(rbuf.cmd.id);

    if (id == 0) {  // GF3-unit Command
      switch (rbuf.cmd.mode) {
        case 0: {
          gf3_.cmd_.read.pending = true;
        } break;
        case 1: {
          gf3_.cmd_.write.pending = true;
        } break;
        default:
          break;
      }
      return;
    }

    if (gf3_.ids_.find(id) == gf3_.ids_.end()) return;

    const auto maybe_saj = utils::SafeAt(gf3_.saj_map_, id);
    if (maybe_saj) {
      auto& cmd = maybe_saj.value()->cmd_;
      cmd.mode = static_cast<SingleAxisJoint::Command::Mode>(rbuf.cmd.mode);
      switch (cmd.mode) {
        case SingleAxisJoint::Command::Mode::Stop: {
          cmd.stop_pending = true;
        } break;
        case SingleAxisJoint::Command::Mode::Fix: {
          cmd.fix_pending = true;
        } break;
        case SingleAxisJoint::Command::Mode::OutPos:
        case SingleAxisJoint::Command::Mode::OutVel: {
          cmd.target_out = static_cast<double>(rbuf.cmd.u.saj.target_out);
          cmd.vel = static_cast<double>(rbuf.cmd.u.saj.vel);
          cmd.max_trq = static_cast<double>(rbuf.cmd.u.saj.max_trq);
          cmd.max_vel = static_cast<double>(rbuf.cmd.u.saj.max_vel);
          cmd.max_acc = static_cast<double>(rbuf.cmd.u.saj.max_acc);
        } break;
        default:
          break;
      }

      return;
    }

    const auto maybe_dj = utils::SafeAt(gf3_.dj_map_, id);
    if (maybe_dj) {
      auto& cmd = maybe_dj.value()->cmd_;
      cmd.mode = static_cast<DifferentialJoint::Command::Mode>(rbuf.cmd.mode);
      switch (cmd.mode) {
        case DifferentialJoint::Command::Mode::Stop: {
          cmd.stop_pending = true;
        } break;
        case DifferentialJoint::Command::Mode::Fix: {
          cmd.fix_pending = true;
        } break;
        case DifferentialJoint::Command::Mode::OutPos:
        case DifferentialJoint::Command::Mode::OutVel: {
          cmd.target_avg = static_cast<double>(rbuf.cmd.u.dj.target_avg);
          cmd.target_dif = static_cast<double>(rbuf.cmd.u.dj.target_dif);
          cmd.vel = static_cast<double>(rbuf.cmd.u.dj.vel);
          cmd.max_trq = static_cast<double>(rbuf.cmd.u.dj.max_trq);
          cmd.max_vel = static_cast<double>(rbuf.cmd.u.dj.max_vel);
          cmd.max_acc = static_cast<double>(rbuf.cmd.u.dj.max_acc);
        } break;
        default:
          break;
      }

      return;
    }
  }

  GF3& gf3_;
};

}  // namespace gf3
