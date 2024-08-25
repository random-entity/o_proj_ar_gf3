#pragma once

#include "saj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> SingleAxisJointFrameMakers::OutVel(SingleAxisJoint* j) {
  auto& cmd = j->cmd_;
  cmd.vel = std::abs(cmd.vel);

  const auto target_pos = std::clamp(cmd.pos, j->min_, j->max_);
  const auto cur_pos = j->s_.GetReplyAux2PositionUncoiled().abs_position;
  const auto target_delta_pos = target_pos - cur_pos;

  auto pm_cmd = *(j->pm_cmd_template_);
  pm_cmd.position = NaN;
  pm_cmd.maximum_torque = cmd.max_trq;
  pm_cmd.velocity_limit = cmd.max_vel;
  pm_cmd.accel_limit = cmd.max_acc;

  if (std::abs(target_delta_pos) >= cmd.fix_thr) {
    pm_cmd.velocity = j->r_ * cmd.vel *
                      std::clamp(target_delta_pos / cmd.damp_thr, -1.0, 1.0);
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
