#pragma once

#include "dj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> DifferentialJointFrameMakers::Fix(
    DifferentialJoint* j) {
  if (j->cmd_.fix_pending) {
    j->cmd_.fix_pending = false;

    const auto cmd = [&] {
      auto c = *(j->pm_cmd_template_);
      c.position = NaN;
      return c;
    }();

    return {j->l_.MakePosition(cmd), j->r_.MakePosition(cmd)};
  } else {
    return {};
  }
}

}  // namespace gf3
