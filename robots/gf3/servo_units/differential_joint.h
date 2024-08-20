#pragma once

#include "../servo.h"

namespace gf3 {
struct DifferentialJointFrameMakers;

// ------------------------------
// | DifferentialJoints on GF3: |
// |----------------------------|
// | Part            | ServoID  |
// | Part            | L  | R   |
// |----------------------------|
// | LeftShoulderXY  | 2  | 3   |
// | LeftElbow       | 4  | 5   |
// | RightShoulderXY | 8  | 9   |
// | RightElbow      | 10 | 11  |
// | Neck            | 13 | 14  |
// ---------------------------------------------------------------------
// | DifferentialJoint formula:                                        |
// |-------------------------------------------------------------------|
// | r_avg * Delta(output_avg) = (Delta(rotor_l) + Delta(rotor_r)) / 2 |
// | r_dif * Delta(output_dif) = (Delta(rotor_l) - Delta(rotor_r)) / 2 |
// ---------------------------------------------------------------------
// | Servo | ID  | aux2 | rot+  | aux2rot+ | sgn(r) |
// |------------------------------------------------|
// | Left  | n   | Dif  | -(-x) | -(-z)    | +      |
// | Right | n+1 | Avg  | +(+x) | +(+x)    | +      |
// --------------------------------------------------

class DifferentialJoint {
  friend struct DifferentialJointFrameMakers;

 public:
  DifferentialJoint(const int& l_id, const int& r_id, const uint8_t& bus,
                    const double& r_avg, const double& r_dif,
                    const double& min_avg, const double& max_avg,
                    const double& min_dif, const double& max_dif)
      : l_{l_id, bus, global_transport, &global_pm_fmt, &global_q_fmt},
        r_{r_id, bus, global_transport, &global_pm_fmt, &global_q_fmt},
        pm_cmd_template_{&global_pm_cmd_template},
        r_avg_{r_avg},
        r_dif_{r_dif},
        min_avg_{min_avg},
        max_avg_{max_avg},
        min_dif_{min_dif},
        max_dif_{max_dif} {}

  /////////////////
  // Components: //

  Servo l_, r_;

  ///////////////////////////////////////
  // DifferentialJoint Command struct: //

  struct Command {
    friend struct DifferentialJointFrameMakers;
    enum class Mode : uint8_t { Stop, OutPos, OutVel, Fix } mode;

    // -------------------------------------------------------------------
    // | Command items                    | Stop | OutPos | OutVel | Fix |
    // |----------------------------------|------------------------------|
    double target_avg, target_dif;     // | X    | O      | O      | X   |
    double vel = 0.0;                  // | X    | X      | O      | X   |
    double max_trq, max_vel, max_acc;  // | X    | O      | O      | X   |
    double damp_threshold = 0.15;      // | X    | X      | O      | X   |
    double fix_threshold = 0.01;       // | X    | O      | O      | X   |
    bool stop_pending;                 // | O    | X      | X      | X   |
    bool fix_pending;                  // | X    | X      | X      | O   |
   private:                            //
    bool fixing;                       // | X    | O      | O      | X   |
    // -------------------------------------------------------------------
  } cmd_;

  const double r_avg_, r_dif_;
  const double min_avg_, max_avg_, min_dif_, max_dif_;
  const PmCmd* const pm_cmd_template_;
};

}  // namespace gf3
