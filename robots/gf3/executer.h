#pragma once

#include "servo_units/gf3.h"
#include "utils/beat.h"

namespace gf3 {

class Executer {
 public:
  Executer(const GF3& gf3, const double& interval = 0.01)
      : gf3_{gf3}, beat_{interval} {}

  void Run() {
    while (1) {
      if (beat_.Hit()) {
        /* Query here or after Transport Cycle? */
        /* Transport Cycle using frames Parsed by ServoUnits */
      }
    }
  }

 private:
  const GF3& gf3_;
  Beat beat_;
};

}  // namespace gf3
