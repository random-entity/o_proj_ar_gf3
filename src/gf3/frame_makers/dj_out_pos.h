#pragma once

#include "dj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> DifferentialJointFrameMakers::OutPos(
    DifferentialJoint* j) {
  auto& cmd = j->cmd_;
  cmd.vel_dif = std::abs(cmd.vel_dif);
  cmd.vel_avg = std::abs(cmd.vel_avg);
  auto& rpl = j->rpl_;

  const auto target_pos_dif =
      std::clamp(cmd.pos_dif, j->min_pos_dif_, j->max_pos_dif_);
  const auto target_pos_avg =
      std::clamp(cmd.pos_avg, j->min_pos_avg_, j->max_pos_avg_);

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

    {
      std::lock_guard lock{rpl.mtx};
      rpl.fixing = false;
    }
  } else if (!cmd.fixing) {
    target_delta_pos_rotor_l = NaN;
    target_delta_pos_rotor_r = NaN;
    cmd.fixing = true;

    {
      std::lock_guard lock{rpl.mtx};
      rpl.fixing = true;
    }
  } else {
    return {};
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
  // pm_cmd.velocity_limit = cmd.max_vel;
  pm_cmd.accel_limit = cmd.max_acc;

  return {j->l_.MakePositionRelativeToRecent([&] {
            auto cmd = pm_cmd;
            cmd.position = target_delta_pos_rotor_l;
            // cmd.position = 0.04 * target_vel_rotor_l;
            cmd.velocity_limit = std::abs(target_vel_rotor_l);
            return cmd;
          }()),
          j->r_.MakePositionRelativeToRecent([&] {
            auto cmd = pm_cmd;
            cmd.position = target_delta_pos_rotor_r;
            // cmd.position = 0.04 * target_vel_rotor_r;
            cmd.velocity_limit = std::abs(target_vel_rotor_r);
            return cmd;
          }())};
}

}  // namespace gf3
