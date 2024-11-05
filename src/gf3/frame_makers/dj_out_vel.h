#pragma once

#include "dj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> DifferentialJointFrameMakers::OutVel(
    DifferentialJoint* j) {
  auto& cmd = j->cmd_;
  auto& rpl = j->rpl_;

  const auto& target_pos_dif = cmd.pos_dif;
  const auto& target_pos_avg = cmd.pos_avg;

  const auto cur_pos_dif = j->l_.GetReplyAux2PositionUncoiled().abs_position;
  const auto cur_pos_avg = j->r_.GetReplyAux2PositionUncoiled().abs_position;

  const auto target_delta_pos_dif = target_pos_dif - cur_pos_dif;
  const auto target_delta_pos_avg = target_pos_avg - cur_pos_avg;

  double target_vel_rotor_l;
  double target_vel_rotor_r;

  if (std::abs(target_delta_pos_dif) >= cmd.fix_thr ||
      std::abs(target_delta_pos_avg) >= cmd.fix_thr) {
    const auto target_vel_dif =
        cmd.vel_dif *
        std::clamp(target_delta_pos_dif / cmd.damp_thr, -1.0, 1.0)
        // Temp low speed for demonstration to Noam
        * 0.675;
    const auto target_vel_avg =
        cmd.vel_avg *
        std::clamp(target_delta_pos_avg / cmd.damp_thr, -1.0, 1.0)
        // Temp low speed for demonstration to Noam
        * 0.675;
    target_vel_rotor_l =
        j->r_avg_ * target_vel_avg + j->r_dif_ * target_vel_dif;
    target_vel_rotor_r =
        j->r_avg_ * target_vel_avg - j->r_dif_ * target_vel_dif;
    cmd.fixing = false;
  }
  // else if (!cmd.fixing) {
  // Just keep fixing to utilize the watchdog timeout.
  else {
    target_vel_rotor_l = 0.0;
    target_vel_rotor_r = 0.0;
    cmd.fixing = true;
  }

  {
    std::lock_guard lock{rpl.mtx};
    rpl.fixing = cmd.fixing;
    rpl.target.vel_rotor.l = target_vel_rotor_l;
    rpl.target.vel_rotor.r = target_vel_rotor_r;
  }

  auto pm_cmd = *(j->pm_cmd_template_);
  pm_cmd.position = NaN;
  pm_cmd.watchdog_timeout = 0.1;

  return {j->l_.MakePosition([&] {
            auto cmd = pm_cmd;
            cmd.velocity = target_vel_rotor_l;
            return cmd;
          }()),
          j->r_.MakePosition([&] {
            auto cmd = pm_cmd;
            cmd.velocity = target_vel_rotor_r;
            return cmd;
          }())};
}

}  // namespace gf3
