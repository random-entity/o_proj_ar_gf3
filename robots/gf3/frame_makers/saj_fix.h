#pragma once

#include "saj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> SingleAxisJointFrameMakers::Fix(SingleAxisJoint* j) {
  if (j->cmd_.fix.pending) {
    j->cmd_.fix.pending = false;

    return {j->s_.MakePosition(j->GetPmCmd(NaN))};
  } else {
    return {};
  }
}

}  // namespace gf3
