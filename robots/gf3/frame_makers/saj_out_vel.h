#pragma once

#include "saj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> SingleAxisJointFrameMakers::OutVel(SingleAxisJoint* j) {
  auto& cmd = j->cmd_;
  cmd.vel = std::abs(cmd.vel);
  cmd.damp_threshold = std::abs(cmd.damp_threshold);
  cmd.fix_threshold = std::abs(cmd.fix_threshold);

  auto pm_cmd = *(j->pm_cmd_template_);
  pm_cmd.position = NaN;
  pm_cmd.maximum_torque = cmd.max_trq;
  pm_cmd.velocity_limit = cmd.max_vel;
  pm_cmd.accel_limit = cmd.max_acc;

  const auto target_out = std::clamp(cmd.target_out, j->min_, j->max_);
  const auto cur_out = j->s_.GetReplyAux2PositionUncoiled().abs_position;
  const auto target_delta_out = target_out - cur_out;

  if (std::abs(target_delta_out) >= cmd.fix_threshold) {
    pm_cmd.velocity =
        j->r_ * cmd.vel *
        std::clamp(target_delta_out / cmd.damp_threshold, -1.0, 1.0);
    cmd.fixing = false;
  } else if (!cmd.fixing) {
    pm_cmd.velocity = 0.0;
    cmd.fixing = true;
  } else {
    return {};
  }

  return {j->s_.MakePosition(pm_cmd)};
}

}  // namespace gf3
