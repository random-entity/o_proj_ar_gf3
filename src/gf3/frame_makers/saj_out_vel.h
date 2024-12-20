#pragma once

#include "saj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> SingleAxisJointFrameMakers::OutVel(SingleAxisJoint* j) {
  auto& cmd = j->cmd_;
  auto& rpl = j->rpl_;

  const auto& target_pos_out = cmd.pos_out;
  const auto cur_pos_out = j->s_.GetReplyAux2PositionUncoiled().abs_position;
  const auto target_delta_pos_out = target_pos_out - cur_pos_out;

  double target_vel_rotor;

  if (std::abs(target_delta_pos_out) >= cmd.fix_thr) {
    const auto target_vel_out =
        cmd.vel_out *
        std::clamp(target_delta_pos_out / cmd.damp_thr, -1.0, 1.0)
        // Temp low speed for demonstration to Noam
        * 0.675;
    target_vel_rotor = j->r_ * target_vel_out;
    cmd.fixing = false;
  }
  // else if (!cmd.fixing) {
  // Just keep fixing to utilize the watchdog timeout.
  else {
    target_vel_rotor = 0.0;
    cmd.fixing = true;
    // return {};
  }

  {
    std::lock_guard lock{rpl.mtx};
    rpl.fixing = cmd.fixing;
    rpl.target_rotor.vel = target_vel_rotor;
  }

  auto pm_cmd = *(j->pm_cmd_template_);
  pm_cmd.position = NaN;
  pm_cmd.velocity = target_vel_rotor;

  return {j->s_.MakePosition(pm_cmd)};
}

}  // namespace gf3
