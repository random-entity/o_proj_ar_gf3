#include "som_linuxcppmoteus.hpp"

using namespace som;

int main() {
  ServoSystem servo_system{{{1, 1}, {12, 1}, {4, 1}, {5, 1}, {14, 1}},
                           CmdPosRelTo::cBASE,
                           RplPosRelTo::rBASE,
                           "../config",
                           "../config",
                           true,
                           RplPosRelTo::rABSOLUTE};
  servo_system.StartThread("x");
  servo_system.StartThread("ec");
  sleep(1);
  servo_system.PrintThreadAll();

  char replies[1024] = {};
  for (;; usleep(0.5 * 1e6)) {
    servo_system.GetReplyAll(replies, sizeof(replies));
    printf("%s", replies);
  }

  servo_system.TerminateThreadAll();
  sleep(1);
  return 0;
}
