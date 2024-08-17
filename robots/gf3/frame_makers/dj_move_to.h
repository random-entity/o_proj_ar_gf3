#pragma once

#include "dj_frame_makers.h"

namespace gf3 {

std::vector<CanFdFrame> DifferentialJointFrameMakers::MoveTo(
    DifferentialJoint* j) {
  auto& cmd = j->cmd_.move_to;

  const auto target_avg = std::clamp(cmd.target_avg, j->min_avg_, j->max_avg_);
  const auto target_dif = std::clamp(cmd.target_dif, j->min_dif_, j->max_dif_);

  const auto cur_avg = j->r_.GetReplyAux2PositionUncoiled().abs_position;
  const auto cur_dif = j->l_.GetReplyAux2PositionUncoiled().abs_position;

  const auto target_delta_avg = target_avg - cur_avg;
  const auto target_delta_dif = target_dif - cur_dif;

  double target_delta_l;
  double target_delta_r;

  if (target_delta_avg > cmd.fix_threshold ||
      target_delta_dif > cmd.fix_threshold) {
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

  return {j->l_.MakePositionRelativeToRecent(j->GetPmCmd(target_delta_l)),
          j->r_.MakePositionRelativeToRecent(j->GetPmCmd(target_delta_r))};
}

}  // namespace gf3
