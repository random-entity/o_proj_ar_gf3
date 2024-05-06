#include "servo_system.hpp"

using namespace som;

int main() {
  ServoSystem servo_system{{{1, 1}, {2, 1}, {3, 1}}, 2.0};
  servo_system.Run();

  return 0;
}
