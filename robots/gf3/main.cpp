#include "cmd_rcvrs/udp_cr.h"
#include "executer.h"
#include "rpl_sndrs/udp_rs.h"

using namespace mjbots;
using namespace gf3;

int main(int argc, char** argv) {
  GF3 gf3{};
  Executer executer{gf3};

  const std::string udp_cr_host = argc >= 2 ? argv[1] : "127.0.0.1";
  const int udp_cr_port = argc >= 3 ? std::stoi(argv[1]) : 8888;
  UdpCommandReceiver udp_cr{gf3, udp_cr_host, udp_cr_port};
  if (!udp_cr.Setup()) return 1;
  std::thread udp_cr_thread{[&] {
    while (1) udp_cr.Run();
  }};

  const std::string udp_rs_host = argc >= 4 ? argv[3] : "127.0.0.1";
  const int udp_rs_port = argc >= 5 ? std::stoi(argv[4]) : 5555;
  UdpReplySender udp_rs{gf3, udp_rs_host, udp_rs_port};
  if (!udp_rs.Setup()) return 1;

  utils::Beat beat{0.01};
  while (1) {
    if (beat.Hit()) {
      executer.Run();
      udp_rs.Run();
    }
  }

  udp_cr_thread.join();

  return 0;
}
