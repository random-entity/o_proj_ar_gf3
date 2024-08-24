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
    struct Decoded {  // Following the protocol used at the
                      // MMCA KOAP2023 exhibition.
      uint8_t id;
      float position;
      float velocity;
      float maximum_torque;
      float accel_limit;
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
    std::map<int, bool> recv_map;  // ID -> (Data received for this ID?)
    for (auto id : gf3_.ids_) recv_map.emplace(id, false);

    // Loop until data are received for all IDs
    while (!std::all_of(recv_map.begin(), recv_map.end(),
                        [](const auto& pair) { return pair.second; })) {
      RecvBuf rbuf;

      if (recvfrom(cfg_.sock, rbuf.raw_bytes, sizeof(rbuf.raw_bytes), 0,
                   (sockaddr*)&(cfg_.addr), &(cfg_.addrlen)) < 0) {
        std::cout << "UDP receive error!" << std::endl;
        continue;
      }

      int id = static_cast<int>(rbuf.cmd.id);

      const auto maybe_received = utils::SafeAt(recv_map, id);
      if (!maybe_received || maybe_received.value()) continue;
      if (gf3_.ids_.find(id) == gf3_.ids_.end()) continue;

      if (utils::IsLittleEndian()) {
        for (int i = 1; i + 4 <= sizeof(rbuf.raw_bytes); i += 4) {
          std::reverse(rbuf.raw_bytes + i, rbuf.raw_bytes + i + 4);
        }
      }

      const auto maybe_saj = utils::SafeAt(gf3_.saj_map_, id);
      if (maybe_saj) {
        auto& cmd = maybe_saj.value()->cmd_;
        cmd.mode = SingleAxisJoint::Command::Mode::OutPos;
        cmd.target_out = static_cast<double>(rbuf.cmd.position);
        cmd.max_trq = static_cast<double>(rbuf.cmd.maximum_torque);
        cmd.max_vel = static_cast<double>(rbuf.cmd.velocity);
        cmd.max_acc = static_cast<double>(rbuf.cmd.accel_limit);

        recv_map[id] = true;
        continue;
      }

      const auto maybe_dj = utils::SafeAt(gf3_.dj_map_, id);
      if (maybe_dj) {
        auto& cmd = maybe_dj.value()->cmd_;
        cmd.mode = DifferentialJoint::Command::Mode::OutPos;
        if (gf3_.dj_lids_.find(id) != gf3_.dj_lids_.end()) {
          cmd.target_dif = static_cast<double>(rbuf.cmd.position);
        } else if (gf3_.dj_rids_.find(id) != gf3_.dj_rids_.end()) {
          cmd.target_avg = static_cast<double>(rbuf.cmd.position);
        }
        cmd.max_trq = static_cast<double>(rbuf.cmd.maximum_torque);
        cmd.max_vel = static_cast<double>(rbuf.cmd.velocity);
        cmd.max_acc = static_cast<double>(rbuf.cmd.accel_limit);

        recv_map[id] = true;
        continue;
      }
    }
  }

  GF3& gf3_;
};

}  // namespace gf3
