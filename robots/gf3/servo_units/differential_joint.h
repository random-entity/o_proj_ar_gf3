#pragma once

#include "../servo.h"

namespace gf3 {
struct DifferentialJointFrameMakers;

// ---------------------------------------
// | DifferentialJoints on GF3:          |
// |-------------------------------------|
// | Part            | ServoID  | SUID   |
// | Part            | L  | R   |        |
// |-------------------------------------|
// | LeftShoulderXY  | 2  | 3   | 2, 3   |
// | LeftElbow       | 4  | 5   | 4, 5   |
// | RightShoulderXY | 8  | 9   | 8, 9   |
// | RightElbow      | 10 | 11  | 10, 11 |
// | Neck            | 13 | 14  | 13, 14 |
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
        r_avg_{r_avg},
        r_dif_{r_dif},
        min_avg_{min_avg},
        max_avg_{max_avg},
        min_dif_{min_dif},
        max_dif_{max_dif},
        pm_cmd_template_{&global_pm_cmd_template} {}

  /////////////////
  // Components: //

  Servo l_, r_;

  ///////////////////////////////////////
  // DifferentialJoint Command struct: //

  struct Command {
    friend struct DifferentialJointFrameMakers;
    enum class Mode : uint8_t { Stop, OutPos, OutVel, Fix } mode = Mode::Stop;

    double target_avg = 0.0, target_dif = 0.0;
    double vel_avg = 0.0, vel_dif = 0.0;
    double max_trq = 0.0, max_vel = 0.0, max_acc = 0.0;
    bool stop_pending = false;
    bool fix_pending = false;

   private:
    inline static const double damp_thr = 0.1;
    inline static const double fix_thr = 0.01;
    bool fixing = false;
  } cmd_;

  /////////////////////
  // Configurations: //

  const double r_avg_, r_dif_;
  const double min_avg_, max_avg_, min_dif_, max_dif_;
  const PmCmd* const pm_cmd_template_;

  ////////////////////////////////////////
  // Command serializer & deserializer: //

  friend void to_json(json& j, const DifferentialJoint& dj) {
    j = json{{"suid", dj.l_.GetId()},
             {"target_avg", dj.cmd_.target_avg},
             {"target_dif", dj.cmd_.target_dif},
             {"vel_avg", dj.cmd_.vel_avg},
             {"vel_dif", dj.cmd_.vel_dif},
             {"max_trq", dj.cmd_.max_trq},
             {"max_vel", dj.cmd_.max_vel},
             {"max_acc", dj.cmd_.max_acc}};
  }

  friend void from_json(const json& j, DifferentialJoint& dj) {
    j.at("target_avg").get_to(dj.cmd_.target_avg);
    j.at("target_dif").get_to(dj.cmd_.target_dif);
    j.at("vel_avg").get_to(dj.cmd_.vel_avg);
    j.at("vel_dif").get_to(dj.cmd_.vel_dif);
    j.at("max_trq").get_to(dj.cmd_.max_trq);
    j.at("max_vel").get_to(dj.cmd_.max_vel);
    j.at("max_acc").get_to(dj.cmd_.max_acc);
  }
};

}  // namespace gf3
