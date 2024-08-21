#pragma once

#include "../servo.h"

namespace gf3 {
struct SingleAxisJointFrameMakers;

// -------------------------------------
// | Single axis joints on GF3:        |
// |-----------------------------------|
// | Part           | ServoID   | SUID |
// |-----------------------------------|
// | LeftShoulderZ  | 1         | 1    |
// | LeftWrist      | 6         | 6    |
// | RightShoulderZ | 7         | 7    |
// | RightWrist     | 12        | 12   |
// -------------------------------------
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
    enum class Mode : uint8_t { Stop, OutPos, OutVel, Fix } mode = Mode::Stop;

    double target_out = 0.0;
    double vel = 0.0;
    double max_trq = 0.0, max_vel = 0.0, max_acc = 0.0;
    bool stop_pending = false;
    bool fix_pending = false;

   private:
    inline static const double damp_thr = 0.1;
    inline static const double fix_thr = 0.0025;
    bool fixing = false;

   public:
  } cmd_;

  /////////////////////
  // Configurations: //

  const double r_;
  const double min_, max_;
  const PmCmd* const pm_cmd_template_;

  ////////////////////////////////////////
  // Command serializer & deserializer: //

  friend void to_json(json& j, const SingleAxisJoint& saj) {
    j = json{{"suid", saj.s_.GetId()},  //
             {"target_out", saj.cmd_.target_out},
             {"vel", saj.cmd_.vel},
             {"max_trq", saj.cmd_.max_trq},
             {"max_vel", saj.cmd_.max_vel},
             {"max_acc", saj.cmd_.max_acc}};
  }

  friend void from_json(const json& j, SingleAxisJoint& saj) {
    j.at("target_out").get_to(saj.cmd_.target_out);
    j.at("vel").get_to(saj.cmd_.vel);
    j.at("max_trq").get_to(saj.cmd_.max_trq);
    j.at("max_vel").get_to(saj.cmd_.max_vel);
    j.at("max_acc").get_to(saj.cmd_.max_acc);
  }
};

}  // namespace gf3
