#include "som_linuxcppmoteus.hpp"

using namespace som;

int main() {
  ///////////////////////////////////////////////////////////////
  /// Initialize the servo system.
  ServoSystem servo_system{{{1, 1}, {2, 1}, {3, 1}},
                           "../examples/simple/config"};
  const auto& ids = servo_system.GetIds();
  std::map<int, std::map<CommandType, double>> input;
  std::map<CommandType, double> input_all;

  ///////////////////////////////////////////////////////////////
  /// Follow internal commands for the first 20 seconds.
  servo_system.SetListeningMode(ListeningMode::INTERNAL);

  // Imitate a clock for 10 seconds.
  const double time_init = helpers::GetNow();
  for (int i = 0; helpers::GetNow() - time_init < 10; i++) {
    input_all[CommandType::POSITION] = 0.25 * i;
    servo_system.InputAll(input_all);
    sleep(1);
  }

  // Wave motion for 10 seconds.
  while (helpers::GetNow() - time_init < 20) {
    for (const auto id : ids) {
      input[id][CommandType::POSITION] = NaN;
      input[id][CommandType::VELOCITY] = std::sin(2 * helpers::GetNow() + id);
    }

    servo_system.Input(input);
    usleep(0.01 * 1e6);
  }

  // Stop and set base positions to current positions.
  servo_system.StopAll();
  servo_system.SetBasePositionsAll();

  ///////////////////////////////////////////////////////////////
  // Now suspend main thread termination while listening to
  // external commands coming through standard input.
  servo_system.SetListeningMode(ListeningMode::EXTERNAL);
  while (true);

  return 0;
}
