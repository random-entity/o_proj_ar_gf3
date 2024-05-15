#include "som_linuxcppmoteus.hpp"

using namespace som;

int main() {
  ///////////////////////////////////////////////////////////////
  /// Initialize the ServoSystem,
  /// and get Servo IDs that succeeded at initialization.
  ServoSystem servo_system{{{1, 1}, {2, 1}, {3, 1}}};
  const auto& ids = servo_system.GetIds();

  // Initialize empty internal input vectors. We will fill
  // them up with commands we want to send to the Servos.
  std::map<int, std::map<CommandType, double>> input;
  std::map<CommandType, double> input_all;

  // Initialize empty output buffer. We will let the
  // ServoSystem to fill it up with servo states.
  char output_buffer[512];

  ///////////////////////////////////////////////////////////////
  /// Demonstrate controlling the ServoSystem by internal commands
  /// for the first 20 seconds.

  // Imitate a clock for 10 seconds.
  const double time_init = Utils::GetNow();
  for (int i = 0; Utils::GetNow() - time_init < 10; i++) {
    input_all[CommandType::POSITION] = 0.25 * i;
    input_all[CommandType::VELOCITY] = 0.0;
    servo_system.InputAll(input_all);

    servo_system.GetOutput(output_buffer, sizeof(output_buffer));
    printf("\n%s\n", output_buffer);

    sleep(1);
  }

  // Wave motion for 10 seconds.
  while (Utils::GetNow() - time_init < 20) {
    for (const auto id : ids) {
      input[id][CommandType::POSITION] = NaN;
      input[id][CommandType::VELOCITY] = std::sin(2 * Utils::GetNow() + id);
    }
    servo_system.Input(input);

    servo_system.GetOutput(output_buffer, sizeof(output_buffer));
    printf("\n%s\n", output_buffer);

    usleep(0.01 * 1e6);
  }

  ///////////////////////////////////////////////////////////////
  /// Stop and set base positions to current positions.
  servo_system.StopAll();
  servo_system.SetBasePositionsAll();

  servo_system.GetOutput(output_buffer, sizeof(output_buffer));
  printf("\n%s\n", output_buffer);

  ///////////////////////////////////////////////////////////////
  // Now start the ServoSystem ExternalInputGetter thread
  // to listen to external commands coming through standard input
  // while suspending main thread termination for 2 minutes.
  servo_system.ThreadsManager.at("ext_in")->Start();

  while (Utils::GetNow() - time_init < 140.0) {
    sleep(1);

    servo_system.GetOutput(output_buffer, sizeof(output_buffer));
    printf("\n%s\n", output_buffer);
  }

  servo_system.ThreadsManager.at("ext_in")->Terminate();

  return 0;
}
