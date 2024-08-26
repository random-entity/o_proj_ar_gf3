#pragma once

#include "dj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> DifferentialJointFrameMakers::OutVel(
    DifferentialJoint* j) {
  auto& cmd = j->cmd_;
  cmd.vel_dif = std::abs(cmd.vel_dif);
  cmd.vel_avg = std::abs(cmd.vel_avg);

  const auto target_pos_dif =
      std::clamp(cmd.pos_dif, j->min_pos_dif_, j->max_pos_dif_);
  const auto target_pos_avg =
      std::clamp(cmd.pos_avg, j->min_pos_avg_, j->max_pos_avg_);

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
        std::clamp(target_delta_pos_dif / cmd.damp_thr, -1.0, 1.0);
    const auto target_vel_avg =
        cmd.vel_avg *
        std::clamp(target_delta_pos_avg / cmd.damp_thr, -1.0, 1.0);
    target_vel_rotor_l =
        j->r_avg_ * target_vel_avg + j->r_dif_ * target_vel_dif;
    target_vel_rotor_r =
        j->r_avg_ * target_vel_avg - j->r_dif_ * target_vel_dif;
    cmd.fixing = false;
  } else if (!cmd.fixing) {
    target_vel_rotor_l = 0.0;
    target_vel_rotor_r = 0.0;
    cmd.fixing = true;
  } else {
    return {};
  }

  auto pm_cmd = *(j->pm_cmd_template_);
  pm_cmd.position = NaN;
  pm_cmd.maximum_torque = cmd.max_trq;
  pm_cmd.velocity_limit = cmd.max_vel;
  pm_cmd.accel_limit = cmd.max_acc;

  return {j->l_.MakePosition([&] {
            pm_cmd.velocity = target_vel_rotor_l;
            return pm_cmd;
          }()),
          j->r_.MakePosition([&] {
            pm_cmd.velocity = target_vel_rotor_r;
            return pm_cmd;
          }())};
}

}  // namespace gf3
