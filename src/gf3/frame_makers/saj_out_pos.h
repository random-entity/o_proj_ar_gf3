#pragma once

#include "saj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> SingleAxisJointFrameMakers::OutPos(SingleAxisJoint* j) {
  auto& cmd = j->cmd_;
  auto& rpl = j->rpl_;

  const auto& target_pos_out = cmd.pos_out;
  const auto cur_pos_out = j->s_.GetReplyAux2PositionUncoiled().abs_position;
  const auto target_delta_pos_out = target_pos_out - cur_pos_out;

  double target_delta_pos_rotor;

  if (std::abs(target_delta_pos_out) >= cmd.fix_thr) {
    target_delta_pos_rotor = j->r_ * target_delta_pos_out;
    cmd.fixing = false;
  } else if (!cmd.fixing) {
    target_delta_pos_rotor = NaN;
    cmd.fixing = true;
  } else {
    return {};
  }

  {
    std::lock_guard lock{rpl.mtx};
    rpl.fixing = cmd.fixing;
    rpl.target_rotor.delta_pos =
        std::isfinite(target_delta_pos_rotor) ? target_delta_pos_rotor : 0.0;
  }

  const auto target_vel_out =
      cmd.vel_out * std::clamp(target_delta_pos_out / cmd.damp_thr, -1.0, 1.0);
  const auto target_vel_rotor = j->r_ * target_vel_out;

  return {j->s_.MakePositionRelativeToRecent([&] {
    auto c = *(j->pm_cmd_template_);
    c.position = target_delta_pos_rotor;
    c.velocity = 0.0;
    c.maximum_torque = cmd.max_trq;
    c.velocity_limit = std::min(std::abs(target_vel_rotor), cmd.max_vel);
    c.accel_limit = cmd.max_acc;
    return c;
  }())};
}

}  // namespace gf3
