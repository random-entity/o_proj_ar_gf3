#include "som_linuxcppmoteus.hpp"

using namespace som;

int main() {
  ServoSystem servo_system{{{1, 1}, {2, 1}, {3, 1}}};
  const auto& ids = servo_system.GetIds();
  char output_buffer[256] = {};
  for (double time_init = Utils::GetNow(); Utils::GetNow() - time_init < 1.0;
       usleep(0.1 * 1e6)) {
    servo_system.StopAll();
  }
  servo_system.SetBasePositionsAll();

  servo_system.GetOutput(output_buffer, sizeof(output_buffer));
  printf("Servo ouput:\n%s\n", output_buffer);

  servo_system.ThreadsManager.at("exin")->Start();

  for (;; sleep(1)) {
    servo_system.GetOutput(output_buffer, sizeof(output_buffer));
    printf("Servo ouput:\n%s\n", output_buffer);
  }

  return 0;
}
