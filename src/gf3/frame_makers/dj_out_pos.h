#pragma once

#include "dj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> DifferentialJointFrameMakers::OutPos(
    DifferentialJoint* j) {
  auto& cmd = j->cmd_;
  auto& rpl = j->rpl_;

  const auto& target_pos_dif = cmd.pos_dif;
  const auto& target_pos_avg = cmd.pos_avg;

  const auto cur_pos_dif = j->l_.GetReplyAux2PositionUncoiled().abs_position;
  const auto cur_pos_avg = j->r_.GetReplyAux2PositionUncoiled().abs_position;

  const auto target_delta_pos_dif = target_pos_dif - cur_pos_dif;
  const auto target_delta_pos_avg = target_pos_avg - cur_pos_avg;

  double target_delta_pos_rotor_l;
  double target_delta_pos_rotor_r;

  if (std::abs(target_delta_pos_dif) >= cmd.fix_thr ||
      std::abs(target_delta_pos_avg) >= cmd.fix_thr) {
    target_delta_pos_rotor_l =
        j->r_avg_ * target_delta_pos_avg + j->r_dif_ * target_delta_pos_dif;
    target_delta_pos_rotor_r =
        j->r_avg_ * target_delta_pos_avg - j->r_dif_ * target_delta_pos_dif;
    cmd.fixing = false;
  } else if (!cmd.fixing) {
    target_delta_pos_rotor_l = NaN;
    target_delta_pos_rotor_r = NaN;
    cmd.fixing = true;
  } else {
    return {};
  }

  {
    std::lock_guard lock{rpl.mtx};
    rpl.fixing = cmd.fixing;
    rpl.target.delta_pos_rotor.l = std::isfinite(target_delta_pos_rotor_l)
                                       ? target_delta_pos_rotor_l
                                       : 0.0;
    rpl.target.delta_pos_rotor.r = std::isfinite(target_delta_pos_rotor_r)
                                       ? target_delta_pos_rotor_r
                                       : 0.0;
  }

  const auto target_vel_dif =
      cmd.vel_dif * std::clamp(target_delta_pos_dif / cmd.damp_thr, -1.0, 1.0);
  const auto target_vel_avg =
      cmd.vel_avg * std::clamp(target_delta_pos_avg / cmd.damp_thr, -1.0, 1.0);
  const auto target_vel_rotor_l =
      j->r_avg_ * target_vel_avg + j->r_dif_ * target_vel_dif;
  const auto target_vel_rotor_r =
      j->r_avg_ * target_vel_avg - j->r_dif_ * target_vel_dif;

  auto pm_cmd = *(j->pm_cmd_template_);
  pm_cmd.velocity = 0.0;
  pm_cmd.maximum_torque = cmd.max_trq;
  pm_cmd.accel_limit = cmd.max_acc;

  return {
      j->l_.MakePositionRelativeToRecent([&] {
        auto c = pm_cmd;
        c.position = target_delta_pos_rotor_l;
        c.velocity_limit = std::min(std::abs(target_vel_rotor_l), cmd.max_vel);
        return c;
      }()),
      j->r_.MakePositionRelativeToRecent([&] {
        auto c = pm_cmd;
        c.position = target_delta_pos_rotor_r;
        c.velocity_limit = std::min(std::abs(target_vel_rotor_r), cmd.max_vel);
        return c;
      }())};
}

}  // namespace gf3
