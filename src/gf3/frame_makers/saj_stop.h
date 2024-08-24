#pragma once

#include "saj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> SingleAxisJointFrameMakers::Stop(SingleAxisJoint* j) {
  if (j->cmd_.stop_pending) {
    j->cmd_.stop_pending = false;

    return {j->s_.MakeStop()};
  } else {
    return {};
  }
}

}  // namespace gf3
