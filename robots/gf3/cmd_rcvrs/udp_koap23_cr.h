#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <string>

#include "gf3.h"

namespace gf3 {

class UdpCommandReceiver {
 public:
  UdpCommandReceiver(const GF3& gf3, const std::string& host, const int& port)
      : gf3_{gf3}, udp_config_{.host = host, .port = port} {}

  struct Udp {
    const std::string host;  // Where to receive Commands from.
    const int port;
    int sock;
    sockaddr_in addr;
    socklen_t addrlen_ = sizeof(addr);
  } udp_config_;

  union RecvBuf {
    struct Decoded {  // Following the namings from the Pd patch used in MMCA
                      // KOAP2023 exhibition.
      uint8_t id;
      float position;
      float velocity;
      float maximum_torque;
      float accel_limit;
    } __attribute__((packed)) cmd;
    uint8_t raw_bytes[sizeof(cmd)];
  };

  bool Setup() {
    udp_config_.sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_config_.sock < 0) {
      std::cout << "Failed to create UDP receive socket." << std::endl;
      return false;
    }
    udp_config_.addr.sin_family = AF_INET;
    udp_config_.addr.sin_addr.s_addr = inet_addr(udp_config_.host.c_str());
    udp_config_.addr.sin_port = htons(udp_config_.port);
    if (bind(udp_config_.sock, (struct sockaddr*)&udp_config_.addr,
             sizeof(udp_config_.addr)) < 0) {
      std::cout << "Failed to bind UDP receive socket." << std::endl;
      close(udp_config_.sock);
      return false;
    }
    return true;
  }

  void Run() {
    if (Setup() < 0) {
      while (1) {
        std::cout << "Failed to create UDP receive socket!" << std::endl;
      }
    }

    while (1) {
      ::usleep(1000);

      std::map<int, bool> received;  // ID -> (Data received for this ID?)
      for (auto id : ids_) {
        received[id] = false;
      }
      std::map<int, std::map<CommandItem, double>> cmd;

      /// Inner loop until data are received for all IDs
      while (!std::all_of(received.begin(), received.end(),
                          [](const auto& pair) { return pair.second; })) {
        RecvBuf rbuf;

        ssize_t bytes_received =
            recvfrom(udp_.sock_r, rbuf.raw_bytes, sizeof(rbuf.raw_bytes), 0,
                     (sockaddr*)&(udp_.addr_r), &(udp_.addrlen_r));
        if (bytes_received < 0) {
          std::cout << "UDP receive error!" << std::endl;
          continue;
        }

        int id = static_cast<int>(rbuf.cmd.id);
        if (ids_.find(id) == ids_.end()) continue;
        if (received[id]) continue;

        if (Utils::IsLittleEndian()) {
          for (int i = 1; i + 4 <= sizeof(rbuf.raw_bytes); i += 4) {
            std::reverse(rbuf.raw_bytes + i, rbuf.raw_bytes + i + 4);
          }
        }

        cmd[id][CommandItem::position] = static_cast<double>(rbuf.cmd.position);
        cmd[id][CommandItem::velocity_limit] =
            static_cast<double>(rbuf.cmd.velocity);
        cmd[id][CommandItem::maximum_torque] =
            static_cast<double>(rbuf.cmd.maximum_torque);
        cmd[id][CommandItem::accel_limit] =
            static_cast<double>(rbuf.cmd.accel_limit);

        received[id] = true;
      }

      EmplaceCommands(cmd);
    }

    close(udp_config_.sock);
  }

  const GF3& gf3_;
};

}  // namespace gf3
