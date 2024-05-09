#include "som_linuxcppmoteus.hpp"

using namespace som;

int main() {
  ///////////////////////////////////////////////////////////////
  /// Initialize the servo system.
  ServoSystem servo_system{{{1, 1}, {2, 1}, {3, 1}},
                           "../examples/simple/config"};
  const auto& ids = servo_system.GetIds();

  ///////////////////////////////////////////////////////////////
  /// Follow internal commands for the first 20 seconds.
  servo_system.SetListeningMode(ListeningMode::INTERNAL);
  std::map<int, std::map<CommandType, double>> input;

  // Imitate a clock for 10 seconds.
  const double time_init = helpers::GetNow();
  for (int i = 0; helpers::GetNow() - time_init < 10; i++) {
    for (auto id : ids) {
      input[id][CommandType::POSITION] = 0.25 * i;
    }
    servo_system.Input(input);

    sleep(1);
  }

  // Make waves for 10 seconds.
  while (helpers::GetNow() - time_init < 20) {
    for (auto id : ids) {
      input[id][CommandType::POSITION] = NaN;
      input[id][CommandType::VELOCITY] = std::sin(2 * helpers::GetNow() + id);
    }

    servo_system.Input(input);
    ::usleep(10000);
  }

  // Go back to initial positions and stop.
  for (auto id : ids) {
    input[id][CommandType::POSITION] = 0;
    input[id][CommandType::VELOCITY] = 0;
  }
  servo_system.Input(input);

  ///////////////////////////////////////////////////////////////
  // Now suspend main thread termination while listening to
  // external commands.
  servo_system.SetListeningMode(ListeningMode::EXTERNAL);

  while (true);

  return 0;
}
