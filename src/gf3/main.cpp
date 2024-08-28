#include "cmd_rcvrs/udp_cr.h"
#include "executer.h"
#include "rpl_sndrs/udp_rs.h"

int main(int argc, char** argv) {
  using namespace mjbots;
  using namespace gf3;

  std::cout << "Usage: gf3 "
               "[UDP CommandReceiver port] "
               "[UDP ReplySender host] "
               "[UDP ReplySender port] "
               "[UDP ReplySender period(sec.)] "
               "[Executer period(sec.)]"
            << std::endl;

  GF3 gf3{};
  Executer executer{gf3};

  const std::string udp_cr_host = "0.0.0.0";
  const int udp_cr_port = argc >= 2 ? std::stoi(argv[1]) : 8888;
  UdpCommandReceiver udp_cr{gf3, udp_cr_host, udp_cr_port};
  if (!udp_cr.Setup()) return 1;
  std::thread udp_cr_thread{[&] {
    while (1) udp_cr.Run();  // recvfrom blocks while waiting UDP packets.
  }};

  const std::string udp_rs_host = argc >= 3 ? argv[2] : "192.168.0.200";
  const int udp_rs_port = argc >= 4 ? std::stoi(argv[3]) : 5555;
  const double udp_rs_period = argc >= 5 ? std::stod(argv[4]) : 1.0 / 24.0;
  UdpReplySender udp_rs{gf3, udp_rs_host, udp_rs_port};
  if (!udp_rs.Setup()) return 1;
  std::thread udp_rs_thread{[&] {
    utils::Beat udp_rs_beat{udp_rs_period};
    while (1) {
      if (udp_rs_beat.Hit()) udp_rs.Run();
    }
  }};

  const double executer_period = argc >= 6 ? std::stod(argv[5]) : 0.01;
  utils::Beat executer_beat{executer_period};
  while (1) {
    if (executer_beat.Hit()) executer.Run();
  }

  udp_cr_thread.join();
  udp_rs_thread.join();

  return 0;
}
