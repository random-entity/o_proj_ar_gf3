#pragma once

#include "dj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> DifferentialJointFrameMakers::Stop(
    DifferentialJoint* j) {
  if (j->cmd_.stop.pending) {
    j->cmd_.stop.pending = false;

    return {j->l_.MakeStop(), j->r_.MakeStop()};
  } else {
    return {};
  }
}

}  // namespace gf3
