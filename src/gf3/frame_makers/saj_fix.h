#pragma once

#include "saj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> SingleAxisJointFrameMakers::Fix(SingleAxisJoint* j) {
  // if (j->cmd_.fix_pending) {
  //   j->cmd_.fix_pending = false;

  {
    std::lock_guard lock{j->rpl_.mtx};
    j->rpl_.fixing = true;
    j->rpl_.target_rotor.delta_pos = 0.0;
  }

  return {j->s_.MakePosition([&] {
    auto c = *(j->pm_cmd_template_);
    c.position = NaN;
    c.velocity = 0.0;
    return c;
  }())};
  // } else {
  //   return {};
  // }
}

}  // namespace gf3
