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

  if (cmd.loaded) {
    std::cout << "Pose loaded" << std::endl;
    std::cout << "target_pos_out = " << target_pos_out << std::endl;
    std::cout << "cur_pos_out = " << cur_pos_out << std::endl;
    std::cout << "target_delta_pos_out = " << target_delta_pos_out << std::endl;
  }

  if (std::abs(target_delta_pos_out) >= cmd.fix_thr) {
    const auto target_vel_out =
        cmd.vel_out *
        std::clamp(target_delta_pos_out / cmd.damp_thr, -1.0, 1.0);
    target_vel_rotor = j->r_ * target_vel_out;
    cmd.fixing = false;
  } else if (!cmd.fixing) {
    target_vel_rotor = 0.0;
    cmd.fixing = true;
  } else {
    return {};
  }

  {
    std::lock_guard lock{rpl.mtx};
    rpl.fixing = cmd.fixing;
    rpl.target_rotor.vel = target_vel_rotor;
  }

  if (cmd.loaded) {
    std::cout << "cmd.fixing = " << cmd.fixing << std::endl;
    std::cout << "target_vel_rotor = " << target_vel_rotor << std::endl;
  }

  auto pm_cmd = *(j->pm_cmd_template_);
  pm_cmd.position = NaN;
  pm_cmd.velocity = target_vel_rotor;
  pm_cmd.maximum_torque = cmd.max_trq;
  pm_cmd.velocity_limit = cmd.max_vel;
  pm_cmd.accel_limit = cmd.max_acc;

  return {j->s_.MakePosition(pm_cmd)};
}

}  // namespace gf3
