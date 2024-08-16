#include "slcm_base.hpp"

using namespace som;

int main() {
  ServoSystem servosystem{{{1, 1}}};
  servosystem.RegisterThread();
}
