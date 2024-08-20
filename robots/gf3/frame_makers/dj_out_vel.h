#pragma once

#include "dj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> DifferentialJointFrameMakers::OutVel(
    DifferentialJoint* j) {
  auto& cmd = j->cmd_;
  cmd.vel = std::abs(cmd.vel);
  cmd.damp_threshold = std::abs(cmd.damp_threshold);
  cmd.fix_threshold = std::abs(cmd.fix_threshold);

  auto pm_cmd = *(j->pm_cmd_template_);
  pm_cmd.position = NaN;
  pm_cmd.maximum_torque = cmd.max_trq;
  pm_cmd.velocity_limit = cmd.max_vel;
  pm_cmd.accel_limit = cmd.max_acc;

  const auto target_avg = std::clamp(cmd.target_avg, j->min_avg_, j->max_avg_);
  const auto target_dif = std::clamp(cmd.target_dif, j->min_dif_, j->max_dif_);

  const auto cur_avg = j->r_.GetReplyAux2PositionUncoiled().abs_position;
  const auto cur_dif = j->l_.GetReplyAux2PositionUncoiled().abs_position;

  const auto target_delta_avg = target_avg - cur_avg;
  const auto target_delta_dif = target_dif - cur_dif;

  double target_vel_l;
  double target_vel_r;

  if (std::abs(target_delta_avg) >= cmd.fix_threshold ||
      std::abs(target_delta_dif) >= cmd.fix_threshold) {
    double target_vel_avg =
        cmd.vel * std::clamp(target_delta_avg / cmd.damp_threshold, -1.0, 1.0);
    double target_vel_dif =
        cmd.vel * std::clamp(target_delta_dif / cmd.damp_threshold, -1.0, 1.0);
    target_vel_l = j->r_avg_ * target_vel_avg + j->r_dif_ * target_vel_dif;
    target_vel_r = j->r_avg_ * target_vel_avg - j->r_dif_ * target_vel_dif;
    cmd.fixing = false;
  } else if (!cmd.fixing) {
    target_vel_l = 0.0;
    target_vel_r = 0.0;
    cmd.fixing = true;
  } else {
    return {};
  }

  return {j->l_.MakePositionRelativeToRecent([&] {
            pm_cmd.velocity = target_vel_l;
            return pm_cmd;
          }()),
          j->r_.MakePositionRelativeToRecent([&] {
            pm_cmd.velocity = target_vel_r;
            return pm_cmd;
          }())};
}

}  // namespace gf3
