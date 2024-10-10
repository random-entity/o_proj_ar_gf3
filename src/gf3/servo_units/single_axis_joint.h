#pragma once

#include "../globals.h"
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
 public:
  SingleAxisJoint(const int& id, const uint8_t& bus, const double& r,
                  const double& min_pos, const double& max_pos)
      : s_{id, bus, globals::transport, &globals::pm_fmt, &globals::q_fmt},
        r_{r},
        min_pos_out_{min_pos},
        max_pos_out_{max_pos},
        pm_cmd_template_{&globals::pm_cmd_template} {}

  /////////////////
  // Components: //

  Servo s_;

  //////////////////////////////////////////////
  // SingleAxisJoint Command & Reply structs: //

  struct Command {
    friend struct SingleAxisJointFrameMakers;
    std::mutex mtx;
    bool loaded = false;

    enum class Mode : uint8_t { Stop, OutPos, OutVel, Fix } mode = Mode::Stop;

    // Ensure min/max clamp for position, and non-negativeness for velocity,
    // at time of reception from CommandReceivers.
    double pos_out = 0.0;
    double vel_out = 0.1;
    double max_trq = 16.0, max_vel = 16.0, max_acc = 16.0;  // of rotor.
    bool stop_pending = false;
    bool fix_pending = false;
    inline static const double damp_thr = 0.1;
    inline static const double fix_thr = 0.01;

   private:
    bool fixing = false;
  } cmd_;

  struct Reply {
    std::mutex mtx;

    bool fixing;
    union {
      double delta_pos;
      double vel;
    } target_rotor;
  } rpl_;

  /////////////////////
  // Configurations: //

  const double r_;
  const double min_pos_out_, max_pos_out_;
  const PmCmd* const pm_cmd_template_;

  ////////////////////////////////////////
  // Command serializer & deserializer: //

  friend void to_json(json& j, const SingleAxisJoint& saj) {
    j = json{{"suid", saj.s_.GetId()},       //
             {"pos_out", saj.cmd_.pos_out},  //
             {"vel_out", saj.cmd_.vel_out},  //
             {"max_trq", saj.cmd_.max_trq},  //
             {"max_vel", saj.cmd_.max_vel},  //
             {"max_acc", saj.cmd_.max_acc}};
  }

  friend void from_json(const json& j, SingleAxisJoint& saj) {
    j.at("pos_out").get_to(saj.cmd_.pos_out);
    j.at("vel_out").get_to(saj.cmd_.vel_out);
    // Ignore maxtva values in pose files since some are corrupt.
    // Maintain the default values instead.
    // j.at("max_trq").get_to(saj.cmd_.max_trq);
    // j.at("max_vel").get_to(saj.cmd_.max_vel);
    // j.at("max_acc").get_to(saj.cmd_.max_acc);

    // Just force maxtva to be constant
    saj.cmd_.max_trq = 32.0;
    saj.cmd_.max_vel = 32.0;
    saj.cmd_.max_acc = 32.0;

    // For compatibility with poses saved before changing
    // "where to clamp at" policy.
    saj.cmd_.pos_out =
        std::clamp(saj.cmd_.pos_out, saj.min_pos_out_, saj.max_pos_out_);

    saj.cmd_.loaded = true;
  }
};

}  // namespace gf3
