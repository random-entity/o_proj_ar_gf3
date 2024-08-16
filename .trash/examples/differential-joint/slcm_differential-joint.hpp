#pragma once

#include "slcm_udp.hpp"
#include "som_linuxcppmoteus.hpp"

namespace som {

class DifferentialJointUdpServoSystem : public UdpServoSystem {
 public:
  DifferentialJointUdpServoSystem(const std::string& host_dest,
                                  const int port_r,
                                  const int port_s)
      : UdpServoSystem{{{4, 1}, {5, 1}},
                       "0.0.0.0",
                       host_dest,
                       port_r,
                       port_s,
                       CommandPositionRelativeTo::Recent,
                       ReplyPositionRelativeTo::Absolute,
                       "../config",
                       "../config",
                       true,
                       ReplyPositionRelativeTo::Absolute} {}

 protected:
  virtual void ExternalCommandGetter(std::atomic_bool* terminated) override {
    std::cout << "Differential-joint variant ExternalCommandGetter thread is "
                 "running..."
              << std::endl;

    const auto& maybe_servo_l = Utils::SafeAt(servos_, 4);
    if (!maybe_servo_l) {
      std::cout << "Servo ID 4 not ready.  Now terminating." << std::endl;
      return;
    }
    const auto& servo_l = maybe_servo_l.value();
    const auto& maybe_servo_r = Utils::SafeAt(servos_, 5);
    if (!maybe_servo_r) {
      std::cout << "Servo ID 5 not ready.  Now terminating." << std::endl;
      return;
    }
    const auto& servo_r = maybe_servo_r.value();

    if (SetupUdpReceive() < 0) {
      std::cout << "Failed to create UDP socket.  "
                   "Differential-joint variant ExternalCommandGetter thread "
                   "will now terminate."
                << std::endl;
      return;
    }

    std::cout << "Differential-joint variant ExternalCommandGetter thread "
                 "started listening for UDP packets on "
              << udp_.host_src << ":" << udp_.port_r << "..." << std::endl;

    /// Listen for UDP packets in an infinite loop
    while (!terminated->load()) {
      ::usleep(cycle_period_us_);

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

      const double target_diff = cmd[4][CommandItem::position];
      const double target_avg = cmd[5][CommandItem::position];
      const double cur_diff = servo_l->GetReply().abs_position;
      const double cur_avg = servo_r->GetReply().abs_position;
      const double target_delta_diff = target_diff - cur_diff;
      const double target_delta_avg = target_avg - cur_avg;

      cmd[4][CommandItem::position] =
          41.0 * 127.0 / 92.0 *
          (target_delta_avg + 145.0 / 127.0 * target_delta_diff);
      cmd[5][CommandItem::position] =
          41.0 * 127.0 / 92.0 *
          (target_delta_avg - 145.0 / 127.0 * target_delta_diff);

      EmplaceCommands(cmd);
    }

    close(udp_.sock_r);
  }
};

}  // namespace som