#pragma once

#include "saj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> SingleAxisJointFrameMakers::Stop(SingleAxisJoint* j) {
  if (j->cmd_.stop_pending) {
    j->cmd_.stop_pending = false;

    {
      std::lock_guard lock{j->rpl_.mtx};
      j->rpl_.fixing = false;
      j->rpl_.target_rotor.delta_pos = 0.0;
    }

    return {j->s_.MakeStop()};
  } else {
    return {};
  }
}

}  // namespace gf3
