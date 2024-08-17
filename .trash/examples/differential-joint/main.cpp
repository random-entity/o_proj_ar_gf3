#include "slcm_differential-joint.hpp"

using namespace som;

int main() {
  std::string host_dest = "192.168.0.25";
  // std::string host_src = "0.0.0.0";
  const int udp_receive_port = 5555;
  const int udp_send_port = 8888;

  ///////////////////////////////////////////////////////////////
  /// Initialize the ServoSystem, and check if the Servos of
  /// right IDs are initialized.
  DifferentialJointUdpServoSystem servo_system{host_dest, udp_receive_port,
                                               udp_send_port};
  const auto& ids = servo_system.GetIds();

  servo_system.SetBasePosition({1, 6});
  servo_system.SetCommandPositionRelativeTo({1, 6},
                                            CommandPositionRelativeTo::Base);
  servo_system.SetReplyPositionRelativeTo({1, 6},
                                          ReplyPositionRelativeTo::Base);

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
