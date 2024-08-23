#pragma once

#include "dj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> DifferentialJointFrameMakers::OutPos(
    DifferentialJoint* j) {
  auto& cmd = j->cmd_;

  const auto target_dif = std::clamp(cmd.target_dif, j->min_dif_, j->max_dif_);
  const auto target_avg = std::clamp(cmd.target_avg, j->min_avg_, j->max_avg_);

  const auto cur_dif = j->l_.GetReplyAux2PositionUncoiled().abs_position;
  const auto cur_avg = j->r_.GetReplyAux2PositionUncoiled().abs_position;

  const auto target_delta_dif = target_dif - cur_dif;
  const auto target_delta_avg = target_avg - cur_avg;

  double target_delta_l;
  double target_delta_r;

  if (std::abs(target_delta_dif) >= cmd.fix_thr ||
      std::abs(target_delta_avg) >= cmd.fix_thr) {
    target_delta_l =
        j->r_avg_ * target_delta_avg + j->r_dif_ * target_delta_dif;
    target_delta_r =
        j->r_avg_ * target_delta_avg - j->r_dif_ * target_delta_dif;
    cmd.fixing = false;
  } else if (!cmd.fixing) {
    target_delta_l = NaN;
    target_delta_r = NaN;
    cmd.fixing = true;
  } else {
    return {};
  }

  auto pm_cmd = *(j->pm_cmd_template_);
  pm_cmd.velocity = 0.0;
  pm_cmd.maximum_torque = cmd.max_trq;
  pm_cmd.velocity_limit = cmd.max_vel;
  pm_cmd.accel_limit = cmd.max_acc;

  return {j->l_.MakePositionRelativeToRecent([&] {
            pm_cmd.position = target_delta_l;
            return pm_cmd;
          }()),
          j->r_.MakePositionRelativeToRecent([&] {
            pm_cmd.position = target_delta_r;
            return pm_cmd;
          }())};
}

}  // namespace gf3
