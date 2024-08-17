#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <cstring>

#include "som_linuxcppmoteus.hpp"

namespace som {

///                         Commands
///                        |--------|
///       |----------|---->| Port R |---->|----------------|
///       |          |     |--------|     |                |
///       | Pd Patch |         UDP        | UdpServoSystem |
///       |          |     |--------|     |                |
///       |----------|<----| Port S |<----|----------------|
///                        |--------|
///                         Replies
///
/// This ServoSystem variant can be used for the following scenario:
/// An external program (probably a Pure Data patch for GUI capability)
/// sends UDP packets that represent commands to send to the Servos to a port,
/// named `port_r` in this script. The ServoSystem receives the UDP
/// packets through the same port, translates them into Servo commands, and
/// sends them to the Servos. The Servos reply their current state, and the
/// ServoSystem translates those replies back to UDP packets then sends them to
/// a port named `port_s` here. An external program (probably the same Pure
/// Data patch) receives them from the same port and translates them to a format
/// suitable to monitor the Servos.
class UdpServoSystem : public ServoSystem {
 protected:
  struct UdpConfig {
    const std::string host_src;   // Where to receive commands from.
    const std::string host_dest;  // Where to send replies to.
    const int port_r;
    const int port_s;
    int sock_r;
    sockaddr_in addr_r;
    socklen_t addrlen_r = sizeof(addr_r);
    int sock_s;
    sockaddr_in addr_s;
  } udp_;

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

  union SendBuf {
    struct Encoded {  // Following the namings from the Pd patch used in MMCA
                      // KOAP2023 exhibition.
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

 public:
  UdpServoSystem(const std::map<int, int>& id_bus_map,
                 const std::string& udp_host_src,
                 const std::string& udp_host_dest,  //
                 const int udp_port_r, const int udp_port_s,
                 const CommandPositionRelativeTo cmd_pos_rel_to =
                     CommandPositionRelativeTo::Base,
                 const ReplyPositionRelativeTo rpl_pos_rel_to =
                     ReplyPositionRelativeTo::Base,
                 const std::string& cmd_conf_dir = "../config",
                 const std::string& rpl_conf_dir = "../config",
                 const bool use_aux2 = true,
                 const ReplyPositionRelativeTo rpl_aux2_pos_rel_to =
                     ReplyPositionRelativeTo::Absolute)
      : ServoSystem{id_bus_map,         cmd_pos_rel_to, rpl_pos_rel_to,
                    cmd_conf_dir,       rpl_conf_dir,   use_aux2,
                    rpl_aux2_pos_rel_to},
        udp_{
            .host_src = udp_host_src,
            .host_dest = udp_host_dest,
            .port_r = udp_port_r,
            .port_s = udp_port_s,
        } {}

 protected:
  bool SetupUdpReceive() {
    udp_.sock_r = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_.sock_r < 0) {
      std::cout << "Failed to create UDP receive socket." << std::endl;
      return false;
    }
    udp_.addr_r.sin_family = AF_INET;
    udp_.addr_r.sin_addr.s_addr = inet_addr(udp_.host_src.c_str());
    udp_.addr_r.sin_port = htons(udp_.port_r);
    if (bind(udp_.sock_r, (struct sockaddr*)&udp_.addr_r, sizeof(udp_.addr_r)) <
        0) {
      std::cout << "Failed to bind UDP receive socket." << std::endl;
      close(udp_.sock_r);
      return false;
    }
    return true;
  }

  virtual void ExternalCommandGetter(std::atomic_bool* terminated) override {
    std::cout << "UDP variant ExternalCommandGetter thread is running..."
              << std::endl;

    if (SetupUdpReceive() < 0) {
      std::cout
          << "Failed to create UDP receive socket.  "
             "UDP variant ExternalCommandGetter thread will now terminate."
          << std::endl;
      return;
    }

    std::cout
        << "UDP variant ExternalCommandGetter thread started listening for "
           "UDP packets on "
        << udp_.host_src << ":" << udp_.port_r << "..." << std::endl;

    /// Listen for UDP packets in an infinite loop
    while (!terminated->load()) {
      ::usleep(cycle_period_us_);

      if (!listen_.external) continue;

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

    close(udp_.sock_r);
  }

  bool SetupUdpSend() {
    udp_.sock_s = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_.sock_s < 0) {
      std::cout << "Failed to create UDP send socket." << std::endl;
      return false;
    }
    struct sockaddr_in addr;
    udp_.addr_s.sin_family = AF_INET;
    udp_.addr_s.sin_addr.s_addr = inet_addr(udp_.host_dest.c_str());
    udp_.addr_s.sin_port = htons(udp_.port_s);
    return true;
  }

  void ExternalReplySender(std::atomic_bool* terminated) override {
    std::cout << "UDP variant ExternalReplySender thread is running..."
              << std::endl;

    if (SetupUdpSend() < 0) {
      std::cout << "Failed to create UDP send socket. "
                   "UDP variant ExternalReplySender thread will now terminate."
                << std::endl;
      return;
    }

    std::cout << "UDP variant ExternalReplySender thread started sending "
                 "UDP packets to "
              << udp_.host_dest << ":" << udp_.port_s << "..." << std::endl;

    while (!terminated->load()) {
      ::usleep(cycle_period_us_);

      for (const auto& id_servo : servos_) {
        const int id = id_servo.first;
        const auto& servo = id_servo.second;
        moteus::Query::Result rpl;
        SendBuf sbuf;
        {
          std::lock_guard<std::mutex> lock(servo_access_mutex_);
          rpl = servo->GetReply();
        }
        sbuf.rpl.id = static_cast<uint8_t>(id);
        sbuf.rpl.position = static_cast<float>(rpl.position);
        sbuf.rpl.aux2_position = static_cast<float>(rpl.abs_position);
        sbuf.rpl.velocity = static_cast<float>(rpl.velocity);
        sbuf.rpl.torque = static_cast<float>(rpl.torque);
        sbuf.rpl.q_curr = static_cast<float>(rpl.q_current);
        sbuf.rpl.d_curr = static_cast<float>(rpl.d_current);
        sbuf.rpl.voltage = static_cast<float>(rpl.voltage);
        sbuf.rpl.temperature = static_cast<float>(rpl.motor_temperature);

        if (Utils::IsLittleEndian()) {
          for (int i = 4; i + 4 <= sizeof(sbuf.raw_bytes); i += 4) {
            std::reverse(sbuf.raw_bytes + i, sbuf.raw_bytes + i + 4);
          }
        }

        sendto(udp_.sock_s, static_cast<void*>(sbuf.raw_bytes), sizeof(sbuf), 0,
               (struct sockaddr*)&(udp_.addr_s), sizeof(udp_.addr_s));
      }
    }

    close(udp_.sock_s);
  }
};

}  // namespace som
