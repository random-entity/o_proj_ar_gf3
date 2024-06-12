#include "slcm_udp.hpp"
#include "som_linuxcppmoteus.hpp"

using namespace som;

int main() {
  ///////////////////////////////////////////////////////////////
  /// Initialize the UdpServoSystem.
  UdpServoSystem servo_system{{{4, 1}, {5, 1}}, "127.0.0.1", 5555, 8888};
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
