#pragma once

#include "saj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> SingleAxisJointFrameMakers::OutPos(SingleAxisJoint* j) {
  auto& cmd = j->cmd_;
  cmd.fix_threshold = std::abs(cmd.fix_threshold);

  const auto target_out = std::clamp(cmd.target_out, j->min_, j->max_);
  const auto cur_out = j->s_.GetReplyAux2PositionUncoiled().abs_position;
  const auto target_delta_out = target_out - cur_out;

  double target_delta_rotor;
  if (std::abs(target_delta_out) >= cmd.fix_threshold) {
    target_delta_rotor = j->r_ * target_delta_out;
    cmd.fixing = false;
  } else if (!cmd.fixing) {
    target_delta_rotor = NaN;
    cmd.fixing = true;
  } else {
    return {};
  }

  return {j->s_.MakePositionRelativeToRecent([&] {
    auto c = *(j->pm_cmd_template_);
    c.position = target_delta_rotor;
    c.velocity = 0.0;
    c.maximum_torque = cmd.max_trq;
    c.velocity_limit = cmd.max_vel;
    c.accel_limit = cmd.max_acc;
    return c;
  }())};
}

}  // namespace gf3
