#include "som_linuxcppmoteus.hpp"

using namespace som;

int main() {
  ServoSystem servo_system{{{1, 1}, {2, 1}, {3, 1}}};
  servo_system.StartThread("r");
  servo_system.StartThread("ei");
  sleep(1);
  servo_system.PrintThreads();

  char output_buffer[256] = {};
  for (;; sleep(1)) {
    servo_system.GetOutput(output_buffer, sizeof(output_buffer));
    printf("%s", output_buffer);
  }

  servo_system.TerminateThreadAll();
  servo_system.PrintThreads();
  sleep(1);
  servo_system.PrintThreads();
  return 0;
}
