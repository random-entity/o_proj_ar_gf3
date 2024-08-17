// Do NOT use this often!

#pragma once

#include "../servo_units/gf3.h"

namespace gf3::utils {

void Zero(GF3& gf3) {
  for (const auto& s : gf3.servos_set_) {
    s->SetStop();
    usleep(0.1e6);
    s->DiagnosticCommand("d cfg-set-output 0");
    usleep(0.1e6);
    s->DiagnosticCommand("conf write");
    usleep(0.1e6);
  }

  for (const auto& s : gf3.servos_set_) s->SetPosition(global_pm_cmd_template);
}

}  // namespace gf3::utils
