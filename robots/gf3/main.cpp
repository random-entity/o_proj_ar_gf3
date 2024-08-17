#include "cmd_rcvrs/udp_koap23_cr.h"
#include "executer.h"
#include "rpl_sndrs/udp_koap23_rs.h"

using namespace mjbots;
using namespace gf3;

int main(int argc, char** argv) {
  GF3 gf3{};

  Executer executer{gf3};

  UdpCommandReceiver udp_cr{gf3, "localhost", 8888};
  if (!udp_cr.Setup()) return 1;

  UdpReplySender udp_rs{gf3, "localhost", 5555};
  if (!udp_rs.Setup()) return 1;

  utils::Beat beat{0.01};

  while (1) {
    if (beat.Hit()) {
      udp_cr.Run();
      executer.Run();
      udp_rs.Run();
    }
  }

  return 0;
}
