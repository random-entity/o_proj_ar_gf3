#pragma once

#include "saj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> SingleAxisJointFrameMakers::MoveTo(SingleAxisJoint* j) {
  auto& cmd = j->cmd_.move_to;

  const auto target_out = cmd.target_out;
  const auto cur_out = j->s_.GetReplyAux2PositionUncoiled().abs_position;
  const auto target_delta_out = target_out - cur_out;

  double target_delta_rotor;
  if (target_delta_out > cmd.fix_threshold) {
    target_delta_rotor = j->r_ * target_delta_out;

    cmd.fixing = false;
  } else if (!cmd.fixing) {
    target_delta_rotor = NaN;

    cmd.fixing = true;
  } else {
    return {};
  }

  return {j->s_.MakePositionRelativeToRecent(j->GetPmCmd(target_delta_rotor))};
}

}  // namespace gf3
