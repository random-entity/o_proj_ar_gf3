#pragma once

#include "dj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> DifferentialJointFrameMakers::Fix(
    DifferentialJoint* j) {
  // if (j->cmd_.fix_pending) {
  //   j->cmd_.fix_pending = false;

  {
    std::lock_guard lock{j->rpl_.mtx};
    j->rpl_.fixing = true;
    j->rpl_.target.delta_pos_rotor.l = 0.0;
    j->rpl_.target.delta_pos_rotor.r = 0.0;
  }

  const auto cmd = [&] {
    auto c = *(j->pm_cmd_template_);
    c.position = NaN;
    c.velocity = 0.0;
    return c;
  }();

  return {j->l_.MakePosition(cmd), j->r_.MakePosition(cmd)};
  // } else {
  //   return {};
  // }
}

}  // namespace gf3
