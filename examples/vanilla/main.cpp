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
  char output_buffer[256] = {};

  ///////////////////////////////////////////////////////////////
  /// Demonstrate controlling the ServoSystem by internal commands
  /// for the first 10 + 10 seconds.

  // Imitate a clock for 10 seconds.
  double time_init = Utils::GetNow();
  for (int i = 0; Utils::GetNow() - time_init < 10; i++, sleep(1)) {
    input_all[CommandType::POSITION] = 0.25 * i;
    input_all[CommandType::VELOCITY] = 0.0;
    servo_system.InputAll(input_all);

    servo_system.GetOutput(output_buffer, sizeof(output_buffer));
    printf("Servo ouput:\n%s\n", output_buffer);
  }

  // Wave motion for 10 seconds.
  for (time_init = Utils::GetNow(); Utils::GetNow() - time_init < 10;
       usleep(0.01 * 1e6)) {
    for (const auto id : ids) {
      input[id][CommandType::POSITION] = NaN;
      input[id][CommandType::VELOCITY] = std::sin(2 * Utils::GetNow() + id);
    }
    servo_system.Input(input);

    servo_system.GetOutput(output_buffer, sizeof(output_buffer));
    printf("Servo ouput:\n%s\n", output_buffer);
  }

  ///////////////////////////////////////////////////////////////
  /// Stop and set base positions to current positions.
  /// Send stop command for 1 second to be sure.
  for (time_init = Utils::GetNow(); Utils::GetNow() - time_init < 1.0;
       usleep(0.1 * 1e6)) {
    servo_system.StopAll();
  }
  servo_system.SetBasePositionsAll();

  servo_system.GetOutput(output_buffer, sizeof(output_buffer));
  printf("Servo ouput:\n%s\n", output_buffer);

  ///////////////////////////////////////////////////////////////
  // Now start the ServoSystem ExternalInputGetter thread
  // to listen to external commands coming through standard input
  // while suspending main thread termination for 1 minutes.
  servo_system.ThreadsManager.at("ei")->Start();

  for (time_init = Utils::GetNow(); Utils::GetNow() - time_init < 60.0;
       sleep(1)) {
    servo_system.GetOutput(output_buffer, sizeof(output_buffer));
    printf("Servo ouput:\n%s\n", output_buffer);
  }

  servo_system.ThreadsManager.at("ei")->Terminate();

  return 0;
}
