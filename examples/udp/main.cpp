#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <cstring>

#include "som_linuxcppmoteus.hpp"

namespace som {

///                           commands
///                          |--------|
///         |----------|---->| Port R |---->|----------------|
///         |          |     |--------|     |                |
///         | Pd Patch |         UDP        | UdpServoSystem |
///         |          |     |--------|     |                |
///         |----------|<----| Port S |<----|----------------|
///                          |--------|
///                            states
///
/// This ServoSystem variant can be used for the following scenario:
/// An external program (probably a Pure Data patch for GUI capability)
/// sends UDP packets that represent commands to send to the servos to a port,
/// named `receive_port` in this script. The ServoSystem receives the UDP
/// packets through the same port, translates them into servo commands, and
/// sends them to the servos. The servos reply their current state, and the
/// ServoSystem translates those replies back to UDP packets then sends them to
/// a port named `send_port` here. An external program (probably the same Pure
/// Data patch) receives them from the same port and translates them to a format
/// suitable to monitor the servos.
class UdpServoSystem : public ServoSystem {
 private:
  struct Udp {
    const std::string host;
    const int receive_port;
    const int send_port;
  } udp_;

  union ReceiveBuffer {
    struct decoded {
      uint8_t id;
      float position;
      float velocity;
      float maximum_torque;
      float velocity_limit;
    } __attribute__((packed)) command;
    uint8_t raw_bytes[sizeof(command)];
  };

  union SendBuffer {
    struct encoded {
      uint8_t id;
      uint16_t false_code;
      uint8_t rezero;
      float position;
      float i2c_position;
      float velocity;
      float torque;
      float q_curr;
      float d_curr;
      float voltage;
      float temperature;
    } __attribute__((packed)) state;
    uint8_t raw_bytes[sizeof(state)];
  };

 public:
  UdpServoSystem(const std::map<int, int>& id_bus_map,
                 const std::string& config_dir_path,
                 const std::string& udp_host, const int udp_receive_port,
                 const int udp_send_port)
      : ServoSystem(id_bus_map, config_dir_path),
        udp_{
            .host = udp_host,
            .receive_port = udp_receive_port,
            .send_port = udp_send_port,
        } {}

 private:
  void ExternalInputGetter(std::atomic_bool* terminated) override {
    std::cout << "UDP variant ExternalInputGetter thread is running..."
              << std::endl;

    /// Setup UDP socket
    int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
      std::cout << "Failed to create UDP socket. "
                   "UDP variant ExternalInputGetter thread will terminate."
                << std::endl;
      return;
    }
    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(udp_.host.c_str());
    address.sin_port = htons(udp_.receive_port);
    if (bind(udp_socket, (struct sockaddr*)&address, sizeof(address)) < 0) {
      std::cout << "Failed to bind UDP socket.  "
                   "UDP variant ExternalInputGetter thread will terminate."
                << std::endl;
      close(udp_socket);
      return;
    }
    std::cout << "UDP variant ExternalInputGetter thread started listening for "
                 "UDP packets on "
              << udp_.host << ":" << udp_.receive_port << "..." << std::endl;

    /// Listen for UDP packets in an infinite loop
    while (!(*terminated).load()) {
      ::usleep(cycle_period_us_);

      std::map<int, bool> receive_states;  // ID -> (Data received for this ID?)
      for (auto id : ids_) {
        receive_states[id] = false;
      }
      std::map<int, std::map<CommandType, double>> input;

      /// Inner loop until data are received for all IDs
      while (!std::all_of(receive_states.begin(), receive_states.end(),
                          [](const auto& pair) { return pair.second; })) {
        ReceiveBuffer buffer;

        sockaddr_in client_address;
        socklen_t client_address_len = sizeof(client_address);
        ssize_t bytes_received =
            recvfrom(udp_socket, buffer.raw_bytes, sizeof(buffer.raw_bytes), 0,
                     (struct sockaddr*)&client_address, &client_address_len);
        if (bytes_received < 0) {
          std::cout << "UDP receive error!" << std::endl;
          continue;
        }

        int id = static_cast<int>(buffer.command.id);
        if (ids_.find(id) == ids_.end()) continue;
        if (receive_states[id]) continue;

        if (Utils::IsLittleEndian()) {
          for (int i = 1; i + 4 <= sizeof(buffer.raw_bytes); i += 4) {
            std::reverse(buffer.raw_bytes + i, buffer.raw_bytes + i + 4);
          }
        }

        input[id][CommandType::POSITION] =
            static_cast<double>(buffer.command.position);
        input[id][CommandType::VELOCITY] =
            static_cast<double>(buffer.command.velocity);
        input[id][CommandType::MAXIMUM_TORQUE] =
            static_cast<double>(buffer.command.maximum_torque);
        input[id][CommandType::VELOCITY_LIMIT] =
            static_cast<double>(buffer.command.velocity_limit);

        receive_states[id] = true;
      }

      InputGetter(input);
    }

    close(udp_socket);
  }

  void ExternalOutputSender(std::atomic_bool* terminated) override {
    std::cout << "UDP variant ExternalOutputSender thread is running..."
              << std::endl;

    /// Setup UDP socket
    int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket == -1) {
      std::cout << "Failed to create UDP socket. "
                   "UDP variant ExternalOutputSender thread will terminate."
                << std::endl;
      return;
    }
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(udp_.host.c_str());
    addr.sin_port = htons(udp_.send_port);
    std::cout << "UDP variant ExternalOutputSender thread started sending "
                 "UDP packets to "
              << udp_.host << ":" << udp_.send_port << "..." << std::endl;

    while (!(*terminated).load()) {
      ::usleep(cycle_period_us_);

      for (const auto& id_servo : servos_) {
        const int id = id_servo.first;
        const auto& servo = id_servo.second;
        const auto& state = servo->state_.recent_reply;

        SendBuffer buffer;

        {
          std::lock_guard<std::mutex> lock(servo_access_mutex_);
          buffer.state.id = static_cast<uint8_t>(id);
          buffer.state.position = state.position;
          buffer.state.velocity = state.velocity;
          buffer.state.torque = state.torque;
          buffer.state.q_curr = state.q_current;
          buffer.state.d_curr = state.d_current;
          buffer.state.voltage = state.voltage;
          buffer.state.temperature = state.temperature;
        }

        if (Utils::IsLittleEndian) {
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

}  // namespace som

using namespace som;

int main() {
  std::string host = "127.0.0.1";
  const int udp_receive_port = 5555;
  const int udp_send_port = 8888;

  ///////////////////////////////////////////////////////////////
  /// Initialize the servo system,
  /// and get Servo IDs that succeeded at initialization.
  UdpServoSystem udp_servo_system{{{1, 1}, {2, 1}, {3, 1}},
                                  "../config",
                                  host,
                                  udp_receive_port,
                                  udp_send_port};
  sleep(1);

  ///////////////////////////////////////////////////////////////
  // Suspend main thread termination while listening to
  // external commands coming through UDP.
  udp_servo_system.ThreadsManager.at("exin")->Start();
  udp_servo_system.ThreadsManager.at("exout")->Start();
  while (true) sleep(1);

  return 0;
}
