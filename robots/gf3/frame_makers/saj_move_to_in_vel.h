#pragma once

#include "saj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> SingleAxisJointFrameMakers::MoveToInVel(
    SingleAxisJoint* j) {
  auto& cmd = j->cmd_.move_to_in_vel;
  cmd.vel = std::abs(cmd.vel);
  cmd.damp_threshold = std::abs(cmd.damp_threshold);
  cmd.fix_threshold = std::abs(cmd.fix_threshold);

  auto pm_cmd = *(j->pm_cmd_template_);
  pm_cmd.position = NaN;
  pm_cmd.maximum_torque = cmd.max_trq;
  pm_cmd.velocity_limit = cmd.max_vel;
  pm_cmd.accel_limit = cmd.max_acc;

  const auto target_out = cmd.target_out;
  const auto cur_out = j->s_.GetReplyAux2PositionUncoiled().abs_position;
  const auto target_delta_out = target_out - cur_out;

  auto& target_vel_rotor = pm_cmd.velocity;

  if (target_delta_out >= cmd.damp_threshold) {
    target_vel_rotor = j->r_ * cmd.vel;
  } else if (target_delta_out <= -cmd.damp_threshold) {
    target_vel_rotor = j->r_ * -cmd.vel;
  } else {
    target_vel_rotor = j->r_ * cmd.vel * target_delta_out / cmd.damp_threshold;
  }

  if (std::abs(target_delta_out) >= cmd.fix_threshold) {
    cmd.fixing = false;
  } else if (!cmd.fixing) {
    target_vel_rotor = 0.0;
  } else {
    return {};
  }

  return {j->s_.MakePosition(pm_cmd)};
}

}  // namespace gf3