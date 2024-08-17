#pragma once

#include "../servo.h"

namespace gf3 {
struct SingleAxisJointFrameMakers;

// ------------------------------
// | Single axis joints on GF3: |
// |----------------------------|
// | Part           | ServoID   |
// |----------------------------|
// | LeftShoulderZ  | 1         |
// | LeftWrist      | 6         |
// | RightShoulderZ | 7         |
// | RightWrist     | 12        |
// ------------------------------------
// | Single axis joint formula:       |
// |----------------------------------|
// | r * Delta(output) = Delta(rotor) |
// ------------------------------------

class SingleAxisJoint {
  friend struct SingleAxisJointFrameMakers;

 public:
  SingleAxisJoint(const int& id, const uint8_t& bus, const double& r,
                  const double& min, const double& max)
      : s_{id, bus, global_transport, &global_pm_fmt, &global_q_fmt},
        r_{r},
        min_{min},
        max_{max},
        pm_cmd_template_{&global_pm_cmd_template} {}

  /////////////////
  // Components: //

  Servo s_;

  /////////////////////////////////////
  // SingleAxisJoint Command struct: //

  struct Command {
    enum class Mode { MoveTo, Stop, Fix } mode;

    struct MoveTo {
      double target_out;
      double max_trq, max_vel, max_acc;
      double fix_threshold = 0.01;

     private:
      friend struct SingleAxisJointFrameMakers;

      bool fixing;
    } move_to;

    struct Stop {
      bool pending = false;
    } stop;

    struct Fix {
      bool pending = false;
    } fix;
  } cmd_;

 private:
  /////////////////////
  // Configurations: //

  const double r_;
  const double min_, max_;
  const PmCmd* const pm_cmd_template_;

 public:
  /////////////////////
  // Helper methods: //

  PmCmd GetPmCmd(const double& pos) {
    auto pm_cmd = *pm_cmd_template_;
    pm_cmd.position = pos;
    pm_cmd.maximum_torque = cmd_.move_to.max_trq;
    pm_cmd.velocity_limit = cmd_.move_to.max_vel;
    pm_cmd.accel_limit = cmd_.move_to.max_acc;
    return pm_cmd;
  }
};

}  // namespace gf3
