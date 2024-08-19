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
    enum class Mode { Stop, MoveTo, MoveToInVel, Fix } mode;

    struct Stop {
      bool pending = false;
    } stop;

    struct MoveTo {
      double target_out;
      double fix_threshold = 0.01;
      double max_trq, max_vel, max_acc;

     private:
      friend struct SingleAxisJointFrameMakers;
      bool fixing;
    } move_to;

    struct MoveToInVel {
      double target_out;
      double vel = 0.0;
      double damp_threshold = 0.1;
      double fix_threshold = 0.01;
      double max_trq, max_vel, max_acc;

     private:
      friend struct SingleAxisJointFrameMakers;
      bool fixing;
    } move_to_in_vel;

    struct Fix {
      bool pending = false;
    } fix;
  } cmd_;

  const double r_;
  const double min_, max_;
  const PmCmd* const pm_cmd_template_;
};

}  // namespace gf3
