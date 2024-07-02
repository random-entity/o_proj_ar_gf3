#include "slcm_udp.hpp"
#include "som_linuxcppmoteus.hpp"

using namespace som;

int main() {
  ///////////////////////////////////////////////////////////////
  /// Initialize the UdpServoSystem.
  UdpServoSystem servo_system{
      {{4, 1}, {5, 1}}, "192.168.0.6", "0.0.0.0", 5555, 8888};
  sleep(1);

  ///////////////////////////////////////////////////////////////
  /// Suspend main thread termination while listening to
  /// external commands and sending replies through UDP.
  servo_system.StartThreadAll();
  while (true);

  ///////////////////////////////////////////////////////////////
  /// Terminate the ServoSystem.
  servo_system.TerminateThreadAll();
  sleep(1);
  return 0;
}
