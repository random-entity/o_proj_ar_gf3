#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <cstring>

#include "som_linuxcppmoteus.hpp"

namespace som {
class Child : public ServoSystem {
 protected:
  struct Udp {
    const std::string host;
    const int recv_port;
    const int send_port;
  } udp_;

  union RecvBuf {
    struct Decoded {
      uint8_t id;
      float position;
      float velocity;
      float maximum_torque;
      float accel_limit;
    } __attribute__((packed)) cmd;
    uint8_t raw_bytes[sizeof(cmd)];
  };

  union SendBuf {
    struct Encoded {
      uint8_t id;
      uint16_t false_code;  // Dummy to match SOM Pd patch.
      uint8_t rezero;       // Dummy to match SOM Pd patch.
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
  Child(const std::map<int, int>& id_bus_map, const std::string& udp_host,
        const int udp_recv_port, const int udp_send_port,
        const CmdPosRelTo cmd_pos_rel_to = CmdPosRelTo::cmdBASE,
        const RplPosRelTo rpl_pos_rel_to = RplPosRelTo::rplBASE,
        const std::string& cmd_conf_dir = "../config",
        const std::string& rpl_conf_dir = "../config",
        const bool use_aux2 = true,
        const RplPosRelTo rpl_aux2_pos_rel_to = RplPosRelTo::rplABSOLUTE)
      : ServoSystem{id_bus_map,         cmd_pos_rel_to, rpl_pos_rel_to,
                    cmd_conf_dir,       rpl_conf_dir,   use_aux2,
                    rpl_aux2_pos_rel_to},
        udp_{
            .host = udp_host,
            .recv_port = udp_recv_port,
            .send_port = udp_send_port,
        } {}

  virtual void ExternalCommandGetter(std::atomic_bool* terminated) override {
    std::cout << "UDP variant ExternalCommandGetter thread is running..."
              << std::endl;

    const auto& maybe_servo_l = Utils::SafeAt(servos_, 4);
    const auto& maybe_servo_r = Utils::SafeAt(servos_, 5);
    if (!maybe_servo_l) {
      std::cout << "Servo ID 4 not ready.  Now terminating." << std::endl;
      return;
    }
    const auto& servo_l = maybe_servo_l.value();
    if (!maybe_servo_r) {
      std::cout << "Servo ID 5 not ready.  Now terminating." << std::endl;
      return;
    }
    const auto& servo_r = maybe_servo_r.value();

    /// Setup UDP socket
    int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
      std::cout
          << "Failed to create UDP socket. "
             "UDP variant ExternalCommandGetter thread will now terminate."
          << std::endl;
      return;
    }
    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(udp_.host.c_str());
    address.sin_port = htons(udp_.recv_port);
    if (bind(udp_socket, (struct sockaddr*)&address, sizeof(address)) < 0) {
      std::cout
          << "Failed to bind UDP socket.  "
             "UDP variant ExternalCommandGetter thread will now terminate."
          << std::endl;
      close(udp_socket);
      return;
    }
    std::cout
        << "UDP variant ExternalCommandGetter thread started listening for "
           "UDP packets on "
        << udp_.host << ":" << udp_.recv_port << "..." << std::endl;

    /// Listen for UDP packets in an infinite loop
    while (!((*terminated).load())) {
      ::usleep(cycle_period_us_);

      std::map<int, bool> receive_states;  // ID -> (Data received for this ID?)
      for (auto id : ids_) {
        receive_states[id] = false;
      }
      std::map<int, std::map<CmdItem, double>> cmd;

      /// Inner loop until data are received for all IDs
      while (!std::all_of(receive_states.begin(), receive_states.end(),
                          [](const auto& pair) { return pair.second; })) {
        RecvBuf buffer;

        sockaddr_in client_address;
        socklen_t client_address_len = sizeof(client_address);
        ssize_t bytes_received =
            recvfrom(udp_socket, buffer.raw_bytes, sizeof(buffer.raw_bytes), 0,
                     (struct sockaddr*)&client_address, &client_address_len);
        if (bytes_received < 0) {
          std::cout << "UDP receive error!" << std::endl;
          continue;
        }

        int id = static_cast<int>(buffer.cmd.id);
        if (ids_.find(id) == ids_.end()) continue;
        if (receive_states[id]) continue;

        if (Utils::IsLittleEndian()) {
          for (int i = 1; i + 4 <= sizeof(buffer.raw_bytes); i += 4) {
            std::reverse(buffer.raw_bytes + i, buffer.raw_bytes + i + 4);
          }
        }

        cmd[id][CmdItem::POSITION] = static_cast<double>(buffer.cmd.position);
        cmd[id][CmdItem::VELOCITY_LIMIT] =
            static_cast<double>(buffer.cmd.velocity);
        cmd[id][CmdItem::MAXIMUM_TORQUE] =
            static_cast<double>(buffer.cmd.maximum_torque);
        cmd[id][CmdItem::ACCEL_LIMIT] =
            static_cast<double>(buffer.cmd.accel_limit);

        receive_states[id] = true;
      }

      const double target_diff = cmd[4][CmdItem::POSITION];
      const double target_avg = cmd[5][CmdItem::POSITION];
      const double cur_diff = servo_l->GetReply().abs_position;
      const double cur_avg = servo_r->GetReply().abs_position;
      const double target_delta_diff = target_diff - cur_diff;
      const double target_delta_avg = target_avg - cur_avg;
      cmd[4][CmdItem::POSITION] =
          41.0 * 127.0 / 92.0 *
          (target_delta_avg + 145.0 / 127.0 * target_delta_diff);
      cmd[5][CmdItem::POSITION] =
          41.0 * 127.0 / 92.0 *
          (target_delta_avg - 145.0 / 127.0 * target_delta_diff);

      EmplaceCommand(cmd);
    }

    close(udp_socket);
  }

  void ExternalReplySender(std::atomic_bool* terminated) override {
    std::cout << "UDP variant ExternalReplySender thread is running..."
              << std::endl;

    /// Setup UDP socket
    int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket == -1) {
      std::cout << "Failed to create UDP socket. "
                   "UDP variant ExternalReplySender thread will now terminate."
                << std::endl;
      return;
    }
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(udp_.host.c_str());
    addr.sin_port = htons(udp_.send_port);
    std::cout << "UDP variant ExternalReplySender thread started sending "
                 "UDP packets to "
              << udp_.host << ":" << udp_.send_port << "..." << std::endl;

    while (!((*terminated).load())) {
      ::usleep(cycle_period_us_);
      if (!listen_.external) continue;

      for (const auto& id_servo : servos_) {
        const int id = id_servo.first;
        const auto& servo = id_servo.second;
        const auto& rpl = servo->GetReply();

        SendBuf buffer;

        {
          std::lock_guard<std::mutex> lock(servo_access_mutex_);
          buffer.rpl.id = static_cast<uint8_t>(id);
          buffer.rpl.position = static_cast<float>(rpl.position);
          buffer.rpl.aux2_position = static_cast<float>(rpl.abs_position);
          buffer.rpl.velocity = static_cast<float>(rpl.velocity);
          buffer.rpl.torque = static_cast<float>(rpl.torque);
          buffer.rpl.q_curr = static_cast<float>(rpl.q_current);
          buffer.rpl.d_curr = static_cast<float>(rpl.d_current);
          buffer.rpl.voltage = static_cast<float>(rpl.voltage);
          buffer.rpl.temperature = static_cast<float>(rpl.temperature);
        }

        if (Utils::IsLittleEndian()) {
          for (int i = 4; i + 4 <= sizeof(buffer.raw_bytes); i += 4) {
            std::reverse(buffer.raw_bytes + i, buffer.raw_bytes + i + 4);
          }
        }

        sendto(udp_socket, static_cast<void*>(buffer.raw_bytes), sizeof(buffer),
               0, (struct sockaddr*)&addr, sizeof(addr));
      }
    }

    close(udp_socket);
  }
};

class Grandchild : public Child {
 public:
  Grandchild(const std::map<int, int>& id_bus_map, const std::string& udp_host,
             const int udp_recv_port, const int udp_send_port,
             const CmdPosRelTo cmd_pos_rel_to = CmdPosRelTo::cmdBASE,
             const RplPosRelTo rpl_pos_rel_to = RplPosRelTo::rplBASE,
             const std::string& cmd_conf_dir = "../config",
             const std::string& rpl_conf_dir = "../config",
             const bool use_aux2 = true,
             const RplPosRelTo rpl_aux2_pos_rel_to = RplPosRelTo::rplABSOLUTE)
      : Child{id_bus_map,     udp_host,       udp_recv_port, udp_send_port,
              cmd_pos_rel_to, rpl_pos_rel_to, cmd_conf_dir,  rpl_conf_dir,
              true,           rpl_pos_rel_to} {}
};
}  // namespace som

using namespace som;

int main() {
  std::string host = "127.0.0.1";
  const int udp_recv_port = 5555;
  const int udp_send_port = 8888;

  ///////////////////////////////////////////////////////////////
  /// Initialize the UdpServoSystem.
  Child servo_system{{{4, 1}, {5, 1}}, host, udp_recv_port, udp_send_port};
  sleep(1);

  ///////////////////////////////////////////////////////////////
  /// Suspend main thread termination while listening to
  /// external commands and sending replies through UDP.
  servo_system.StartThreadAll();
  while (true) sleep(1);

  ///////////////////////////////////////////////////////////////
  /// Terminate the ServoSystem.
  servo_system.TerminateThreadAll();
  sleep(1);
  return 0;
}
