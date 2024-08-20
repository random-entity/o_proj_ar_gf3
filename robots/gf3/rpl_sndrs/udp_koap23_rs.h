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
    struct Encoded {  // Following the protocol used at the
                      // MMCA KOAP2023 exhibition.
      uint8_t id;
      uint16_t false_code;
      uint8_t rezero;
      float position;
      float aux2_position;
      float velocity;
      float torque;
      float q_curr;
      float d_curr;
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
      sbuf.rpl.false_code = 0x1234;
      // Added: Encoder validity, temporarily using "rezero" slot.
      sbuf.rpl.rezero = static_cast<uint8_t>(rpl.extra[0].value);
      sbuf.rpl.position = static_cast<float>(rpl.position);
      sbuf.rpl.aux2_position = static_cast<float>(rpl.abs_position);
      sbuf.rpl.velocity = static_cast<float>(rpl.velocity);
      sbuf.rpl.torque = static_cast<float>(rpl.torque);
      sbuf.rpl.q_curr = static_cast<float>(rpl.q_current);
      sbuf.rpl.d_curr = static_cast<float>(rpl.d_current);
      sbuf.rpl.voltage = static_cast<float>(rpl.voltage);
      sbuf.rpl.temperature = static_cast<float>(rpl.motor_temperature);

      if (utils::IsLittleEndian()) {
        for (int i = 4; i + 4 <= sizeof(sbuf.raw_bytes); i += 4) {
          std::reverse(sbuf.raw_bytes + i, sbuf.raw_bytes + i + 4);
        }
      }

      sendto(cfg_.sock, static_cast<void*>(sbuf.raw_bytes), sizeof(sbuf), 0,
             (struct sockaddr*)&(cfg_.addr), sizeof(cfg_.addr));
    }
  }

  GF3& gf3_;
};

}  // namespace gf3
