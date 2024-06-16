#pragma once

#include "slcm_udp.hpp"
#include "som_linuxcppmoteus.hpp"

namespace som {
class DifferentialJointUdpServoSystem : public UdpServoSystem {
 public:
  DifferentialJointUdpServoSystem(const std::string& udp_host,
                                  const int udp_recv_port,
                                  const int udp_send_port)
      : UdpServoSystem{{{4, 1}, {5, 1}},
                       udp_host,
                       udp_recv_port,
                       udp_send_port,
                       CmdPosRelTo::cRECENT,
                       RplPosRelTo::rABSOLUTE,
                       "../config",
                       "../config",
                       true,
                       RplPosRelTo::rABSOLUTE} {}

 protected:
  virtual void ExternalCommandGetter(std::atomic_bool* terminated) override {
    std::cout << "Differential joint variant ExternalCommandGetter thread is "
                 "running..."
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
      std::cout << "Failed to create UDP socket. "
                   "Differential joint variant ExternalCommandGetter thread "
                   "will now terminate."
                << std::endl;
      return;
    }
    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(udp_.host.c_str());
    address.sin_port = htons(udp_.recv_port);
    if (bind(udp_socket, (struct sockaddr*)&address, sizeof(address)) < 0) {
      std::cout << "Failed to bind UDP socket.  "
                   "Differential joint variant ExternalCommandGetter thread "
                   "will now terminate."
                << std::endl;
      close(udp_socket);
      return;
    }
    std::cout << "Differential joint variant ExternalCommandGetter thread "
                 "started listening for UDP packets on "
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

        cmd[id][CmdItems::position] = static_cast<double>(buffer.cmd.position);
        cmd[id][CmdItems::velocity_limit] =
            static_cast<double>(buffer.cmd.velocity);
        cmd[id][CmdItems::maximum_torque] =
            static_cast<double>(buffer.cmd.maximum_torque);
        cmd[id][CmdItems::accel_limit] =
            static_cast<double>(buffer.cmd.accel_limit);

        receive_states[id] = true;
      }

      const double target_diff = cmd[4][CmdItems::position];
      const double target_avg = cmd[5][CmdItems::position];
      const double cur_diff = servo_l->GetReply().abs_position;
      const double cur_avg = servo_r->GetReply().abs_position;
      const double target_delta_diff = target_diff - cur_diff;
      const double target_delta_avg = target_avg - cur_avg;

      cmd[4][CmdItems::position] =
          41.0 * 127.0 / 92.0 *
          (target_delta_avg + 145.0 / 127.0 * target_delta_diff);
      cmd[5][CmdItems::position] =
          41.0 * 127.0 / 92.0 *
          (target_delta_avg - 145.0 / 127.0 * target_delta_diff);

      std::cout << "target_diff = " << target_diff << std::endl;
      std::cout << "target_avg = " << target_avg << std::endl;
      std::cout << "cur_diff = " << cur_diff << std::endl;
      std::cout << "cur_avg = " << cur_avg << std::endl;
      std::cout << "target_delta_diff = " << target_delta_diff << std::endl;
      std::cout << "target_delta_avg = " << target_delta_avg << std::endl;
      std::cout << "target_delta_l = " << cmd[4][CmdItems::position]
                << std::endl;
      std::cout << "target_delta_r = " << cmd[5][CmdItems::position]
                << std::endl;
      EmplaceCommand(cmd);
    }

    close(udp_socket);
  }

  /// TEMPORARY
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
          buffer.rpl.temperature = static_cast<float>(rpl.motor_temperature);
        }

        std::cout << id << " Temperature: " << rpl.motor_temperature
                  << std::endl;

        if (Utils::IsLittleEndian()) {
          for (int i = 4; i + 4 <= sizeof(buffer.raw_bytes); i += 4) {
            std::reverse(buffer.raw_bytes + i, buffer.raw_bytes + i + 4);
          }
        }

        sendto(udp_socket, static_cast<void*>(buffer.raw_bytes), sizeof(buffer),
               0, (struct sockaddr*)&addr, sizeof(addr));
      }

      // 6 test
      SendBuf buffer;
      buffer.rpl.id = static_cast<uint8_t>(6);
      // buffer.rpl.position = static_cast<float>(rpl.position);
      // buffer.rpl.aux2_position = static_cast<float>(rpl.abs_position);
      // buffer.rpl.velocity = static_cast<float>(rpl.velocity);
      // buffer.rpl.torque = static_cast<float>(rpl.torque);
      // buffer.rpl.q_curr = static_cast<float>(rpl.q_current);
      // buffer.rpl.d_curr = static_cast<float>(rpl.d_current);
      // buffer.rpl.voltage = static_cast<float>(rpl.voltage);
      buffer.rpl.temperature = static_cast<float>(12.34);
      if (Utils::IsLittleEndian()) {
        for (int i = 4; i + 4 <= sizeof(buffer.raw_bytes); i += 4) {
          std::reverse(buffer.raw_bytes + i, buffer.raw_bytes + i + 4);
        }
      }

      sendto(udp_socket, static_cast<void*>(buffer.raw_bytes), sizeof(buffer),
             0, (struct sockaddr*)&addr, sizeof(addr));
    }

    close(udp_socket);
  }
};
}  // namespace som