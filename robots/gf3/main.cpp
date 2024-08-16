#include "servo_units/gf3.h"

using namespace mjbots;
using namespace gf3;

int main(int argc, char** argv) {
  Servo s{1, 1, moteus::Controller::MakeSingletonTransport({})};

  return 0;
}
