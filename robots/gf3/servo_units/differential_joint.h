#pragma once

#include <algorithm>
#include <mutex>
#include <string>

#include "../servo.h"
#include "../utils/templates.h"

namespace gf3 {

//-------------------------------------
// Differential joints on GF3:        |
//-------------------------------------
// Part            | LeftID | RightID |
//-------------------------------------
// LeftShoulderXY  | 2      | 3       |
// RightShoulderXY | 4      | 5       |
// LeftElbow       | 8      | 9       |
// RightElbow      | 10     | 11      |
// Neck            | 13     | 14      |
//-------------------------------------------------------------------
// Differential joint formula:                                      |
//-------------------------------------------------------------------
// r_avg  * Delta(out_avg)  = (Delta(rotor_l) + Delta(rotor_r)) / 2 |
// r_diff * Delta(out_diff) = (Delta(rotor_l) - Delta(rotor_r)) / 2 |
//-------------------------------------------------------------------
// Servo | ID  | aux2 | rot   | aux2rot                    | sgn(r) |
//-------------------------------------------------------------------
// Left  | n   | Diff | -(-x) | -(-z)                      | +      |
// Right | n+1 | Avg  | +(+x) | +(+x) if attached on right | +      |
//-------------------------------------------------------------------
// Store Base out_diff and out_avg to l_ and r_'s base_aux2_pos respectively.

class DifferentialJoint {
 public:
  DifferentialJoint(const int& l_id, const int& r_id, const uint8_t& bus,
                    const std::shared_ptr<Transport>& transport,
                    const double& r_avg, const double& r_diff,
                    const double& min_avg, const double& max_avg,
                    const double& min_diff, const double& max_diff)
      : l_{l_id, bus, transport, &global_pm_fmt, &global_q_fmt},
        r_{r_id, bus, transport, &global_pm_fmt, &global_q_fmt},
        r_avg_{r_avg},
        r_diff_{r_diff},
        min_avg_{min_avg},
        max_avg_{max_avg},
        min_diff_{min_diff},
        max_diff_{max_diff},
        pm_cmd_template_{&global_pm_cmd_template} {}

  enum RunningState { Nop, Running } running_state_;

  struct Command {
    // Types of Commands: Continuous, BangThenResume, BangThenNop

    // Continuous:

    struct Move {
      bool pending = false;

      PosRelTo rel;  // NOT handling PosRelTo::Recent.
      double target_avg, target_diff;
      double max_torque, max_vel, max_accel;

      bool operator==(const Move& other) {
        return rel == other.rel &&                  //
               target_avg == other.target_avg &&    //
               target_diff == other.target_diff &&  //
               max_torque == other.max_torque &&    //
               max_vel == other.max_vel &&          //
               max_accel == other.max_accel;
      }

      bool operator!=(const Move& other) { return !(*this == other); }
    } move;

    // BangThenResume:

    struct SetBaseFromCurrent {
      bool pending = false;
      double offset = 0.0;
    } set_base_from_current;

    struct SetBaseToValue {
      bool pending = false;
      double base_l, base_r;
    } set_base_to_value;

    struct SetBaseFromFile {
      bool pending = true;
      std::string path;
    } set_base_from_file;

    struct WriteToFile {
      bool pending = false;
      std::string path;
    } write_to_file;

    // BangThenNop:

    struct Stop {
      bool pending = false;
    } stop;

    struct Brake {
      bool pending = false;
    } brake;

    struct Fix {
      bool pending = false;
    } fix;
  } cmd_;

  std::mutex cmd_mtx_;

  double GetBaseAvg() { return r_.GetBaseAux2Pos(); }

  double GetBaseDiff() { return l_.GetBaseAux2Pos(); }

  void SetBaseFromCurrent(const double& offset_avg = 0.0,
                          const double& offset_diff = 0.0) {
    l_.SetBaseAux2PosFromCurrent(offset_diff);
    r_.SetBaseAux2PosFromCurrent(offset_avg);
  }

  void SetBaseToValue(const double& base_avg, const double& base_diff) {
    l_.SetBaseAux2PosToValue(base_diff);
    r_.SetBaseAux2PosToValue(base_avg);
  }

  PmCmd GetPmCmd(const double& pos, const Command& cmd) {
    auto pm_cmd = *pm_cmd_template_;
    pm_cmd.position = pos;
    pm_cmd.maximum_torque = cmd.move.max_torque;
    pm_cmd.velocity_limit = cmd.move.max_vel;
    pm_cmd.accel_limit = cmd.move.max_accel;
    return pm_cmd;
  }

  std::vector<CanFdFrame> ParseCommand() {
    // Just handle Move Command for now!

    auto target_avg = cmd_.move.target_avg;
    auto target_diff = cmd_.move.target_diff;

    const auto base_avg = GetBaseAvg();
    const auto base_diff = GetBaseDiff();

    const auto base_not_set = std::isnan(base_avg) || std::isnan(base_diff);

    if (cmd_.move.rel == PosRelTo::Base) {
      if (base_not_set) {
        std::cout << "DifferentialJoint: "
                     "You requested a Base-relative Command, "
                     "but Base positions are NOT set.  "
                     "Not sending PositionMode Command."
                  << std::endl;
        return {l_.MakeQuery(), r_.MakeQuery()};
      }

      target_avg += base_avg;
      target_diff += base_diff;
    }

    if (base_not_set) {
      std::cout << "DifferentialJoint: "
                   "Cannot clamp Command position between min, max "
                   "since Base positions are NOT set.  "
                   "Proceeding without clamping."
                << std::endl;
    } else {
      target_avg =
          std::clamp(target_avg, base_avg + min_avg_, base_avg + max_avg_);
      target_diff =
          std::clamp(target_diff, base_diff + min_diff_, base_diff + max_diff_);
    }

    const auto cur_avg = r_.GetRpl().abs_position;
    const auto cur_diff = l_.GetRpl().abs_position;

    const auto target_delta_avg = target_avg - cur_avg;
    const auto target_delta_diff = target_diff - cur_diff;

    double target_delta_l;
    double target_delta_r;

    if (target_delta_avg > 0.01 || target_delta_diff > 0.01) {
      target_delta_l = r_avg_ * target_delta_avg + r_diff_ * target_delta_diff;
      target_delta_r = r_avg_ * target_delta_avg - r_diff_ * target_delta_diff;

      fixing_ = false;
    } else if (!fixing_) {
      target_delta_l = NaN;
      target_delta_r = NaN;

      fixing_ = true;
    } else {
      return {l_.MakeQuery(), r_.MakeQuery()};
    }

    return {l_.GetFrame(GetPmCmd(target_delta_l, cmd_), PosRelTo::Recent),
            r_.GetFrame(GetPmCmd(target_delta_r, cmd_), PosRelTo::Recent)};
  }

  Servo l_, r_;

 private:
  const double r_avg_, r_diff_;
  const double min_avg_, max_avg_, min_diff_, max_diff_;  // Relative to Base.
  bool fixing_;
  const PmCmd* const pm_cmd_template_;
};

}  // namespace gf3
