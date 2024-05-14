#include "som_linuxcppmoteus.hpp"

using namespace som;

int main() {
  ///////////////////////////////////////////////////////////////
  /// Initialize the servo system,
  /// and get servo IDs that succeeded at initialization.
  ServoSystem servo_system{{{1, 1}, {2, 1}, {3, 1}}};
  const auto& ids = servo_system.GetIds();

  // Initialize empty internal input vectors. We will fill
  // them up with commands we want to send to the servos.
  std::map<int, std::map<CommandType, double>> input;
  std::map<CommandType, double> input_all;

  // Initialize empty output buffer. We will let the
  // servo system to fill it up with servo states.
  char output_buffer[512];

  ///////////////////////////////////////////////////////////////
  /// Let the servo system follow internal commands
  /// for the first 20 seconds.
  servo_system.SetListeningMode(ListeningMode::INTERNAL);

  // Imitate a clock for 10 seconds.
  const double time_init = helpers::GetNow();
  for (int i = 0; helpers::GetNow() - time_init < 10; i++) {
    input_all[CommandType::POSITION] = 0.25 * i;
    input_all[CommandType::VELOCITY] = 0.0;
    servo_system.InputAll(input_all);

    servo_system.GetOutput(output_buffer, sizeof(output_buffer));
    printf("\n%s\n", output_buffer);

    sleep(1);
  }

  // Wave motion for 10 seconds.
  while (helpers::GetNow() - time_init < 20) {
    for (const auto id : ids) {
      input[id][CommandType::POSITION] = NaN;
      input[id][CommandType::VELOCITY] = std::sin(2 * helpers::GetNow() + id);
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
  // Now suspend main thread termination while listening to
  // external commands coming through standard input.
  servo_system.SetListeningMode(ListeningMode::EXTERNAL);
  while (true) {
    sleep(1);

    servo_system.GetOutput(output_buffer, sizeof(output_buffer));
    printf("\n%s\n", output_buffer);
  }

  return 0;
}
