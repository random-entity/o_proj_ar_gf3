#include "cmd_rcvrs/udp_cr.h"
#include "executer.h"
#include "rpl_sndrs/udp_rs.h"

using namespace mjbots;
using namespace gf3;

int main(int argc, char** argv) {
  GF3 gf3{};
  Executer executer{gf3};

  UdpCommandReceiver udp_cr{gf3, "127.0.0.1", 8888};
  if (!udp_cr.Setup()) return 1;
  std::thread udp_cr_thread{[&] {
    while (1) udp_cr.Run();
  }};

  UdpReplySender udp_rs{gf3, "127.0.0.1", 5555};
  if (!udp_rs.Setup()) return 1;

  utils::Beat beat{0.01};
  while (1) {
    if (beat.Hit()) {
      executer.Run();
      udp_rs.Run();
    }
  }

  return 0;
}
