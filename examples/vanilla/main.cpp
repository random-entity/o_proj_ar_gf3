#include "som_linuxcppmoteus.hpp"

using namespace som;

int main() {
  ///////////////////////////////////////////////////////////////
  /// Initialize the ServoSystem,
  /// and get Servo IDs that succeeded at initialization.
  ServoSystem servo_system{{{1, 1}}};
  const auto& ids = servo_system.GetIds();

  // Initialize empty internal command maps. We will fill
  // them up with commands we want to send to the Servos.
  std::map<int, std::map<CommandItem, double>> cmd;
  std::map<CommandItem, double> cmd_all;

  // Initialize empty reply buffer. We will let the
  // ServoSystem to fill it up with servo replies.
  char replies[1024] = {};

  // Make sure to start the Executor thread which is responsible for
  // the continuous communication with the ServoSystem.
  servo_system.StartThread("Executor");

  ///////////////////////////////////////////////////////////////
  /// Demonstrate controlling the ServoSystem by internal commands
  /// for the first 10 + 10 seconds. No additional threads are
  /// required for internal command and reply.

  // Imitate a clock for 10 seconds.
  double time_init = Utils::GetTime();
  for (int i = 0; Utils::GetTime() - time_init < 10.0; i++, sleep(1)) {
    cmd_all[CommandItem::position] = 0.25 * i;
    cmd_all[CommandItem::velocity] = 0.0;
    servo_system.CommandAll(cmd_all);

    servo_system.GetReplyAll(replies, sizeof(replies));
    printf("Servo replies:\n%s\n", replies);
  }

  // Wave motion for 10 seconds.
  for (time_init = Utils::GetTime(); Utils::GetTime() - time_init < 10.0;
       usleep(0.01 * 1e6)) {
    for (const auto id : ids) {
      cmd[id][CommandItem::position] = NaN;
      cmd[id][CommandItem::velocity] = std::sin(2 * Utils::GetTime() + id);
    }
    servo_system.Command(cmd);

    servo_system.GetReplyAll(replies, sizeof(replies));
    printf("Servo replies:\n%s\n", replies);
  }

  ///////////////////////////////////////////////////////////////
  /// Stop and set base positions to current positions.
  /// Send stop command for 1 second to make sure the motors
  /// completely stop before setting base positions.
  for (time_init = Utils::GetTime(); Utils::GetTime() - time_init < 1.0;
       usleep(0.1 * 1e6)) {
    servo_system.FixAll();
  }
  servo_system.SetBasePositionAll();

  servo_system.GetReplyAll(replies, sizeof(replies));
  printf("Servo replies:\n%s\n", replies);

  ///////////////////////////////////////////////////////////////
  // Now start the ServoSystem ExternalCommandGetter thread
  // to listen to external commands coming through standard input
  // while suspending main thread termination for 1 minute.
  servo_system.StartThread("ExternalCommandGetter");

  for (time_init = Utils::GetTime(); Utils::GetTime() - time_init < 60.0;
       sleep(1)) {
    servo_system.GetReplyAll(replies, sizeof(replies));
    printf("Servo replies:\n%s\n", replies);
  }

  ///////////////////////////////////////////////////////////////
  /// Terminate the ServoSystem.
  servo_system
      .TerminateThreadAll();  // This may get stuck at std::getline().
                              // Hit Enter on input console
                              // or just use Ctrl-c for now to terminate.
  sleep(1);
  return 0;
}
