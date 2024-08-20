#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "../servo_units/gf3.h"

namespace gf3 {

class UdpReplySender {
 public:
  UdpReplySender(GF3& gf3, const std::string& host, const int& port)
      : gf3_{gf3}, cfg_{.host = host, .port = port} {}

  ~UdpReplySender() { close(cfg_.sock); }

  struct UdpConfig {
    const std::string host;  // Where to send Replies to.
    const int port;
    int sock;
    sockaddr_in addr;
  } cfg_;

  union SendBuf {
    struct Encoded {
      uint8_t id;
      uint8_t mode;
      uint8_t trajectory_complete;
      uint8_t fault;
      uint8_t encoder_validity;
      float position;
      float velocity;
      float torque;
      float aux2_position;
      float power;
      float motor_temperature;
      float voltage;
      float temperature;
    } __attribute__((packed)) rpl;
    uint8_t raw_bytes[sizeof(rpl)];
  };

  bool Setup() {
    cfg_.sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (cfg_.sock < 0) {
      std::cout << "Failed to create UDP send socket." << std::endl;
      return false;
    }
    struct sockaddr_in addr;
    cfg_.addr.sin_family = AF_INET;
    cfg_.addr.sin_addr.s_addr = inet_addr(cfg_.host.c_str());
    cfg_.addr.sin_port = htons(cfg_.port);
    return true;
  }

  void Run() {
    for (const auto& pair : gf3_.servos_map_) {
      const int id = pair.first;
      const auto* servo = pair.second;
      const auto rpl = servo->GetReplyAux2PositionUncoiled();
      SendBuf sbuf;
      sbuf.rpl.id = static_cast<uint8_t>(id);
      sbuf.rpl.mode = static_cast<uint8_t>(rpl.mode);
      sbuf.rpl.position = static_cast<float>(rpl.position);
      sbuf.rpl.velocity = static_cast<float>(rpl.velocity);
      sbuf.rpl.torque = static_cast<float>(rpl.torque);
      sbuf.rpl.aux2_position = static_cast<float>(rpl.abs_position);
      sbuf.rpl.power = static_cast<float>(rpl.power);
      sbuf.rpl.motor_temperature = static_cast<float>(rpl.motor_temperature);
      sbuf.rpl.trajectory_complete =
          static_cast<uint8_t>(rpl.trajectory_complete);
      sbuf.rpl.voltage = static_cast<float>(rpl.voltage);
      sbuf.rpl.temperature = static_cast<float>(rpl.motor_temperature);
      sbuf.rpl.fault = static_cast<uint8_t>(rpl.fault);
      sbuf.rpl.encoder_validity = static_cast<uint8_t>(rpl.extra[0].value);

      sendto(cfg_.sock, static_cast<void*>(sbuf.raw_bytes), sizeof(sbuf), 0,
             (struct sockaddr*)&(cfg_.addr), sizeof(cfg_.addr));
    }
  }

  GF3& gf3_;
};

}  // namespace gf3