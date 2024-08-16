#pragma once

#include <mutex>

#include "../servo.h"
#include "../utils/templates.h"

namespace gf3 {

// ----------------------------
// Single axis joints on GF3: |
// ----------------------------
// Part           | ID        |
// ----------------------------
// LeftShoulderZ  | 2         |
// RightShoulderZ | 6         |
// LeftWrist      | 7         |
// RightWrist     | 12        |
// -------------------------------
// Single axis joint formula:    |
// -------------------------------
// r * Delta(out) = Delta(rotor) |
//--------------------------------
// Store Base out to s_'s base_aux2_pos.

class SingleAxisJoint {
 public:
  SingleAxisJoint(const int& id, const int& r_id, const uint8_t& bus,
                  const std::shared_ptr<Transport>& transport,  //
                  const double& r, const double& min_out, const double& max_out)
      : s_{id, bus, transport, &global_pm_fmt, &global_q_fmt},
        r_{r},
        min_out_{min_out},
        max_out_{max_out},
        pm_cmd_template_{&global_pm_cmd_template} {}

  enum RunningState{Nop, Running};

  struct Command {
    enum class Mode { Move, SetBaseFromCurrent, Stop, Brake, Fix } mode;

    PosRelTo rel;  // NOT handling PosRelTo::Recent.
    double target_out;
    double max_torque, max_vel, max_accel;

    bool operator==(const Command& other) {
      if (mode != other.mode) return false;
      if (mode != Mode::Move) return true;

      return rel == other.rel &&                //
             target_out == other.target_out &&  //
             max_torque == other.max_torque &&  //
             max_vel == other.max_vel &&        //
             max_accel == other.max_accel;
    }

    bool operator!=(const Command& other) { return !(*this == other); }
  } in_cmd_;

  std::mutex in_cmd_mtx_;

  double GetBaseOut() { return s_.GetBaseAux2Pos(); }

  void SetBaseFromCurrent(const double& offset = 0.0) {
    s_.SetBaseAux2PosFromCurrent(offset);
  }

  void SetBaseToValue(const double& base_out) {
    s_.SetBaseAux2PosToValue(base_out);
  }

  PmCmd GetPmCmd(const double& pos, const Command& cmd) {
    auto pm_cmd = *pm_cmd_template_;
    pm_cmd.position = pos;
    pm_cmd.maximum_torque = cmd.max_torque;
    pm_cmd.velocity_limit = cmd.max_vel;
    pm_cmd.accel_limit = cmd.max_accel;
    return pm_cmd;
  }

  std::vector<CanFdFrame> ParseCommand() {
    // TODO: Handle Modes other than Move.

    {
      std::lock_guard lock{in_cmd_mtx_};
      if (cmd_ != in_cmd_) cmd_ = in_cmd_;
    }

    auto target_out = cmd_.target_out;

    if (cmd_.rel == PosRelTo::Base) {
      const auto base_out = GetBaseOut();

      if (std::isnan(base_out)) {
        std::cout << "SingleAxisJoint: "
                     "You requested a Base-relative Command, "
                     "but Base positions are NOT set. "
                     "Not sending PositionMode Command."
                  << std::endl;
        return {s_.MakeQuery()};
      } else {
        target_out += base_out;
      }
    }

    const auto cur_out = s_.GetRpl().abs_position;

    const auto target_delta_out = target_out - cur_out;

    double target_delta_rotor;

    if (target_delta_out > 0.01) {
      target_delta_rotor = r_ * target_delta_out;

      fixing_ = false;
    } else if (!fixing_) {
      target_delta_rotor = NaN;

      fixing_ = true;
    } else {
      return {s_.MakeQuery()};
    }

    return {s_.GetFrame(GetPmCmd(target_delta_rotor, cmd_), PosRelTo::Recent)};
  }

  Servo s_;

 private:
  const double r_;
  const double min_out_, max_out_;
  Command cmd_;
  bool fixing_;
  const PmCmd* const pm_cmd_template_;
};

}  // namespace gf3
