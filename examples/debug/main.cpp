#include "som_linuxcppmoteus.hpp"

using namespace som;

int main() {
  ServoSystem servo_system{{{1, 1}, {2, 1}, {3, 1}}};
  servo_system.ThreadsManager.at("exin")->Start();
  char output_buffer[256] = {};

  for (;; sleep(1)) {
    servo_system.GetOutput(output_buffer, sizeof(output_buffer));
    printf("ServoSystem ouput:\n%s\n", output_buffer);
  }

  return 0;
}
