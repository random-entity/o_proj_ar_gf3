#include "som_linuxcppmoteus.hpp"

using namespace som;

int main() {
  ServoSystem servo_system{{{1, 1}, {2, 1}, {3, 1}}};
  servo_system.Run();

  return 0;
}
