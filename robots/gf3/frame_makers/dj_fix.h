#pragma once

#include "dj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> DifferentialJointFrameMakers::Fix(
    DifferentialJoint* j) {
  if (j->cmd_.fix.pending) {
    j->cmd_.fix.pending = false;

    return {j->l_.MakePosition(j->GetPmCmd(NaN)),
            j->r_.MakePosition(j->GetPmCmd(NaN))};
  } else {
    return {};
  }
}

}  // namespace gf3
