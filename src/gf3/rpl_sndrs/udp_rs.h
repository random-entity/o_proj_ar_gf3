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

 private:
  struct UdpConfig {
    const std::string host;  // Where to send Replies to.
    const int port;
    int sock;
    sockaddr_in addr;
  } cfg_;

  union ServoRplSendBuf {
    struct Encoded {
      uint8_t rid;
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
      float aux2_velocity;
    } __attribute__((packed)) rpl;
    uint8_t raw_bytes[sizeof(rpl)];
  };

  union SAJRplSendBuf {
    struct Encoded {
      uint8_t rid;
      uint8_t fixing;
      float target_rotor;
    } __attribute__((packed)) rpl;
    uint8_t raw_bytes[sizeof(rpl)];
  };

  union DJRplSendBuf {
    struct Encoded {
      uint8_t rid;
      uint8_t fixing;
      float target_rotor_l;
      float target_rotor_r;
    } __attribute__((packed)) rpl;
    uint8_t raw_bytes[sizeof(rpl)];
  };

  union SAJCRRplSendBuf {
    struct Encoded {
      uint8_t rid;
      float pos_out;
      float vel_out;
      float max_trq, max_vel, max_acc;
    } __attribute__((packed)) rpl;
    uint8_t raw_bytes[sizeof(rpl)];
  };

  union DJCRRplSendBuf {
    struct Encoded {
      uint8_t rid;
      float pos_dif;
      float vel_dif;
      float pos_avg;
      float vel_avg;
      float max_trq, max_vel, max_acc;
    } __attribute__((packed)) rpl;
    uint8_t raw_bytes[sizeof(rpl)];
  };

 public:
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
    std::cout << "UdpReplySender started sending to " << cfg_.host << ":"
              << cfg_.port << "." << std::endl;
    return true;
  }

  void Run() {
    for (const auto& pair : gf3_.servo_map_) {
      const auto id = pair.first;
      auto* servo = pair.second;
      QRpl rpl;
      {
        std::lock_guard lock{servo->mtx_};
        rpl = servo->GetReplyAux2PositionUncoiled();
      }
      ServoRplSendBuf sbuf;
      sbuf.rpl.rid = static_cast<uint8_t>(id);
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
      sbuf.rpl.aux2_velocity = static_cast<float>(rpl.extra[0].value);
      sbuf.rpl.encoder_validity = static_cast<uint8_t>(rpl.extra[1].value);

      sendto(cfg_.sock, static_cast<void*>(sbuf.raw_bytes), sizeof(sbuf), 0,
             (struct sockaddr*)&(cfg_.addr), sizeof(cfg_.addr));
    }

    for (const auto& pair : gf3_.saj_map_) {
      const auto suid = pair.first;
      auto* j = pair.second;
      auto& cmd = j->cmd_;
      auto& rpl = j->rpl_;
      {
        SAJRplSendBuf sbuf;
        {
          std::lock_guard lock{rpl.mtx};
          sbuf.rpl.rid = static_cast<uint8_t>(100 + suid);
          sbuf.rpl.fixing = static_cast<uint8_t>(rpl.fixing);
          sbuf.rpl.target_rotor = static_cast<float>(
              j->cmd_.mode == SingleAxisJoint::Command::Mode::OutVel
                  ? rpl.target_rotor.vel
                  : rpl.target_rotor.delta_pos);
        }
        sendto(cfg_.sock, static_cast<void*>(sbuf.raw_bytes), sizeof(sbuf), 0,
               (struct sockaddr*)&(cfg_.addr), sizeof(cfg_.addr));
      }

      if (j->cmd_.loaded) {
        SAJCRRplSendBuf sbuf;
        {
          std::lock_guard lock{cmd.mtx};
          sbuf.rpl.rid = static_cast<uint8_t>(200 + suid);
          sbuf.rpl.pos_out = static_cast<float>(cmd.pos_out);
          sbuf.rpl.vel_out = static_cast<float>(cmd.vel_out);
          sbuf.rpl.max_trq = static_cast<float>(cmd.max_trq);
          sbuf.rpl.max_vel = static_cast<float>(cmd.max_vel);
          sbuf.rpl.max_acc = static_cast<float>(cmd.max_acc);
        }
        sendto(cfg_.sock, static_cast<void*>(sbuf.raw_bytes), sizeof(sbuf), 0,
               (struct sockaddr*)&(cfg_.addr), sizeof(cfg_.addr));
        j->cmd_.loaded = false;
      }
    }

    for (const auto& pair : gf3_.dj_map_) {
      const auto suid = pair.first;
      auto* j = pair.second;
      auto& cmd = j->cmd_;
      auto& rpl = j->rpl_;
      {
        DJRplSendBuf sbuf;
        {
          std::lock_guard lock{rpl.mtx};
          sbuf.rpl.rid = static_cast<uint8_t>(100 + suid);
          sbuf.rpl.fixing = static_cast<uint8_t>(rpl.fixing);
          sbuf.rpl.target_rotor_l = static_cast<float>(
              j->cmd_.mode == DifferentialJoint::Command::Mode::OutVel
                  ? rpl.target.vel_rotor.l
                  : rpl.target.delta_pos_rotor.l);
          sbuf.rpl.target_rotor_r = static_cast<float>(
              j->cmd_.mode == DifferentialJoint::Command::Mode::OutVel
                  ? rpl.target.vel_rotor.r
                  : rpl.target.delta_pos_rotor.r);
        }
        sendto(cfg_.sock, static_cast<void*>(sbuf.raw_bytes), sizeof(sbuf), 0,
               (struct sockaddr*)&(cfg_.addr), sizeof(cfg_.addr));
      }

      if (j->cmd_.loaded) {
        DJCRRplSendBuf sbuf;
        {
          std::lock_guard lock{cmd.mtx};
          sbuf.rpl.rid = static_cast<uint8_t>(200 + suid);
          sbuf.rpl.pos_dif = static_cast<float>(cmd.pos_dif);
          sbuf.rpl.vel_dif = static_cast<float>(cmd.vel_dif);
          sbuf.rpl.pos_avg = static_cast<float>(cmd.pos_avg);
          sbuf.rpl.vel_avg = static_cast<float>(cmd.vel_avg);
          sbuf.rpl.max_trq = static_cast<float>(cmd.max_trq);
          sbuf.rpl.max_vel = static_cast<float>(cmd.max_vel);
          sbuf.rpl.max_acc = static_cast<float>(cmd.max_acc);
        }
        sendto(cfg_.sock, static_cast<void*>(sbuf.raw_bytes), sizeof(sbuf), 0,
               (struct sockaddr*)&(cfg_.addr), sizeof(cfg_.addr));
        j->cmd_.loaded = false;
      }
    }
  }

 private:
  GF3& gf3_;
};

}  // namespace gf3
