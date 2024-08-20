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
    friend struct SingleAxisJointFrameMakers;
    enum class Mode { Stop, OutPos, OutVel, Fix } mode;

    // -------------------------------------------------------------------
    // | Command items                    | Stop | OutPos | OutVel | Fix |
    // |----------------------------------|------------------------------|
    double target_out;                 // | X    | O      | O      | X   |
    double vel;                        // | X    | X      | O      | X   |
    double damp_threshold = 0.15;      // | X    | X      | O      | X   |
    double fix_threshold = 0.01;       // | X    | O      | O      | X   |
    double max_trq, max_vel, max_acc;  // | X    | O      | O      | X   |
   private:                            //
    bool fixing;                       // | X    | O      | O      | X   |
   public:                             //
    bool stop_pending;                 // | O    | X      | X      | X   |
    bool fix_pending;                  // | X    | X      | X      | O   |
    // -------------------------------------------------------------------
  } cmd_;

  const double r_;
  const double min_, max_;
  const PmCmd* const pm_cmd_template_;
};

}  // namespace gf3
