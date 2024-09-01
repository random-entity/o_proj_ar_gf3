#pragma once

#include "dj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> DifferentialJointFrameMakers::Stop(
    DifferentialJoint* j) {
  if (j->cmd_.stop_pending) {
    j->cmd_.stop_pending = false;

    {
      std::lock_guard lock{j->rpl_.mtx};
      j->rpl_.fixing = false;
      j->rpl_.target.delta_pos_rotor.l = 0.0;
      j->rpl_.target.delta_pos_rotor.r = 0.0;
    }

    return {j->l_.MakeStop(), j->r_.MakeStop()};
  } else {
    return {};
  }
}

}  // namespace gf3
