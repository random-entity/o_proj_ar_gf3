#include "slcm_udp.hpp"
#include "som_linuxcppmoteus.hpp"

using namespace som;

int main() {
  std::string host = "127.0.0.1";
  const int udp_recv_port = 5555;
  const int udp_send_port = 8888;

  ///////////////////////////////////////////////////////////////
  /// Initialize the UdpServoSystem.
  UdpServoSystem servo_system{
      {{4, 1}, {5, 1}}, host, udp_recv_port, udp_send_port};
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
