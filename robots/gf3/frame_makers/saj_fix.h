#pragma once

#include "saj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> SingleAxisJointFrameMakers::Fix(SingleAxisJoint* j) {
  if (j->cmd_.fix.pending) {
    j->cmd_.fix.pending = false;

    return {j->s_.MakePosition([&] {
      auto c = *(j->pm_cmd_template_);
      c.position = NaN;
      return c;
    }())};
  } else {
    return {};
  }
}

}  // namespace gf3
