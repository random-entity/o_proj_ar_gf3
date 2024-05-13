#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

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

 public:
  UdpServoSystem(const std::map<int, int>& id_bus_map = {{1, 0}},
                 const std::string& config_dir_path = ".",
                 const std::string& udp_host, const int udp_receive_port,
                 const int udp_send_port)
      : ServoSystem(id_bus_map, config_dir_path, true, true),
        udp_{
            .host = udp_host,
            .receive_port = udp_receive_port,
            .send_port = udp_send_port,
        } {}

 private:
  void ExternalInputGetter() override {
    // Setup UDP socket
    int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket == -1) {
      std::cerr << "Failed to create UDP socket. "
                   "ExternalInputGetter thread will terminate."
                << std::endl;
      return;
    }
    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(udp_.host.c_str());
    address.sin_port = htons(udp_.receive_port);
    if (bind(udp_socket, (struct sockaddr*)&address, sizeof(address)) == -1) {
      std::cerr << "Failed to bind UDP socket. "
                   "ExternalInputGetter thread will terminate."
                << std::endl;
      close(udp_socket);
      return;
    }
    std::cout
        << "ExternalInputGetter thread started listening for UDP packets on "
        << udp_.host << ":" << udp_.receive_port << "..." << std::endl;

    while (!terminated_) {  // Enter infinite loop to receive UDP packets
      ::usleep(cycle_period_us_);

      std::map<int, bool> receive_states;  // ID -> (data received for this ID?)
      for (auto id : ids_) {
        receive_states[id] = false;
      }

      while (!std::all_of(  // Loop until commands are received for all IDs
          receive_states.begin(), receive_states.end(),
          [](const auto& pair) { return pair.second; })) {
        try {
          std::array<char, 32> data;
          sockaddr_in client_address;
          socklen_t client_address_len = sizeof(client_address);
          ssize_t bytes_received =
              recvfrom(udp_socket, data.data(), data.size(), 0,
                       (struct sockaddr*)&client_address, &client_address_len);
          if (bytes_received == -1) {
            std::cerr << "UDP receive error!" << std::endl;
            continue;
          }

          int received_id = (int)data[0];

          const auto maybe_receive_state =
              helpers::SafeAt(receive_states, received_id);
          if (!maybe_receive_state) continue;
          if (maybe_receive_state.value()) continue;

          const auto maybe_servo = helpers::SafeAt(servos_, received_id);
          if (!maybe_servo) continue;
          const auto& command = maybe_servo.value()->command_;

          {
            std::lock_guard<std::mutex> lock(mutex_);
            std::memcpy((void*)&(command.position), data.data() + 1,
                        sizeof(float));
            std::memcpy((void*)&(command.velocity), data.data() + 5,
                        sizeof(float));
            std::memcpy((void*)&(command.maximum_torque), data.data() + 9,
                        sizeof(float));
            // std::memcpy((void*)&(command.acceleration), data.data() + 13,
            //             sizeof(float));
          }

          receive_states[received_id] = true;
        } catch (...) {
          std::cerr << "UDP receive error!" << std::endl;
        }
      }
    }
    close(udp_socket);
  }

  void ExternalOutputSender() override {
    int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket == -1) {
      // TODO: Handle socket creation error
      return;
    }
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(udp_.send_port);
    addr.sin_addr.s_addr = inet_addr(udp_.host.c_str());

    while (!terminated_) {
      ::usleep(cycle_period_us_);

      for (const auto& id_servo : servos_) {
        const int id = id_servo.first;
        const auto& servo = id_servo.second;
        const auto& state = servo->state_.recent_reply;

        std::vector<uint8_t> num_list;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          num_list = {(uint8_t)id,
                      0,
                      0,  // Assuming placeholders for your data
                      0,  // state.rezero_state,
                      (uint8_t)state.position,
                      0,  // state.i2c_position,
                      (uint8_t)state.velocity,
                      (uint8_t)state.torque,
                      (uint8_t)state.q_current,
                      (uint8_t)state.d_current,
                      (uint8_t)state.voltage,
                      (uint8_t)state.temperature};
        }

        std::vector<uint8_t> packed_data(
            /* sizeof(uint8_t) * */ num_list.size());
        memcpy(packed_data.data(), num_list.data(), packed_data.size());

        sendto(udp_socket, packed_data.data(), packed_data.size(), 0,
               (struct sockaddr*)&addr, sizeof(addr));
      }
    }

    close(udp_socket);
  }
};

}  // namespace som

using namespace som;

int main(int argc, char** argv) {  // Get UDP host as argv[1]
  std::string host = argv[1];
  const int udp_receive_port = std::stoi(argv[2]);
  const int udp_send_port = std::stoi(argv[3]);

  ///////////////////////////////////////////////////////////////
  /// Initialize the servo system,
  /// and get servo IDs that succeeded at initialization.
  UdpServoSystem servo_system{
      {{1, 1}, {2, 1}, {3, 1}}, ".", host, udp_receive_port, udp_send_port};
  const auto& ids = servo_system.GetIds();

  ///////////////////////////////////////////////////////////////
  // Suspend main thread termination while listening to
  // external commands coming through UDP.
  servo_system.SetListeningMode(ListeningMode::EXTERNAL);
  while (true);

  return 0;
}
