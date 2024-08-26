#pragma once

#include "saj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> SingleAxisJointFrameMakers::OutPos(SingleAxisJoint* j) {
  auto& cmd = j->cmd_;

  const auto target_pos = std::clamp(cmd.pos, j->min_pos_, j->max_pos_);
  const auto cur_pos = j->s_.GetReplyAux2PositionUncoiled().abs_position;
  const auto target_delta_pos = target_pos - cur_pos;

  double target_delta_pos_rotor;
  if (std::abs(target_delta_pos) >= cmd.fix_thr) {
    target_delta_pos_rotor = j->r_ * target_delta_pos;
    cmd.fixing = false;
  } else if (!cmd.fixing) {
    target_delta_pos_rotor = NaN;
    cmd.fixing = true;
  } else {
    return {};
  }

  return {j->s_.MakePositionRelativeToRecent([&] {
    auto c = *(j->pm_cmd_template_);
    c.position = target_delta_pos_rotor;
    c.velocity = 0.0;
    c.maximum_torque = cmd.max_trq;
    c.velocity_limit = cmd.max_vel;
    c.accel_limit = cmd.max_acc;
    return c;
  }())};
}

}  // namespace gf3
