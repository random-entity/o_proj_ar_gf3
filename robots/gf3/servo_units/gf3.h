#pragma once

#include "differential_joint.h"
#include "single_axis_joint.h"

namespace gf3 {

// --------------------------------------------------------
// | ServoUnits on GF3:                                   |
// |------------------------------------------------------|
// | Type              | Part            | ServoID | SUID |
// |                   |                 | L  | R  |      |
// |------------------------------------------------------|
// | GF3               | GF3             | 1 ~ 14  | 0    |
// | SingleAxisJoint   | LeftShoulderZ   | 1       | 1    |
// | DifferentialJoint | LeftShoulderXY  | 2  | 3  | 2    |
// | DifferentialJoint | LeftElbow       | 4  | 5  | 4    |
// | SingleAxisJoint   | LeftWrist       | 6       | 6    |
// | SingleAxisJoint   | RightShoulderZ  | 7       | 7    |
// | DifferentialJoint | RightShoulderXY | 8  | 9  | 8    |
// | DifferentialJoint | RightElbow      | 10 | 11 | 10   |
// | SingleAxisJoint   | RightWrist      | 12      | 12   |
// | DifferentialJoint | Neck            | 13 | 14 | 13   |
// --------------------------------------------------------

class GF3 {
 public:
  GF3()
      : l_shoulder_z_{1, 1, r_.sz, -0.25, 0.117},
        l_shoulder_xy_{2, 3, 1, r_.sxya, r_.sxyd, -0.42, 0.389, 0.0, 0.462},
        l_elbow_{4, 5, 1, r_.ea, r_.ed, -0.475, 0.33, -0.08, 0.349},
        l_wrist_{6, 1, r_.wr, -0.23, 0.221},

        // Right arm minmax NOT set!
        r_shoulder_z_{7, 2, r_.sz, -0.5, 0.5},
        r_shoulder_xy_{8, 9, 2, r_.sxya, r_.sxyd, -0.5, 0.5, -0.5, 0.5},
        r_elbow_{10, 11, 2, r_.ea, r_.ed, -0.5, 0.5, -0.5, 0.5},
        r_wrist_{12, 2, r_.wr, -0.5, 0.5},

        neck_{13, 14, 3, r_.na, r_.nd, -0.36, 0.36, -0.22, 0.22},

        saj_set_{&l_shoulder_z_, &l_wrist_, &r_shoulder_z_, &r_wrist_},
        saj_map_{[&] {
          std::map<int, SingleAxisJoint*> js;
          for (const auto& j : saj_set_) js.emplace(j->s_.GetId(), j);
          return js;
        }()},
        dj_set_{&l_shoulder_xy_, &l_elbow_, &r_shoulder_xy_, &r_elbow_, &neck_},
        dj_map_{[&] {
          std::map<int, DifferentialJoint*> js;
          for (const auto& j : dj_set_) js.emplace(j->l_.GetId(), j);
          return js;
        }()},
        servo_set_{[&] {
          std::set<Servo*> servos;
          for (const auto& j : saj_set_) servos.emplace(&j->s_);
          for (const auto& j : dj_set_) {
            servos.emplace(&j->l_);
            servos.emplace(&j->r_);
          }
          return servos;
        }()},
        servo_map_{[&] {
          std::map<int, Servo*> map;
          for (const auto& j : servo_set_) map.emplace(j->GetId(), j);
          return map;
        }()} {}

  /////////////////
  // Components: //

  SingleAxisJoint l_shoulder_z_;
  DifferentialJoint l_shoulder_xy_;
  DifferentialJoint l_elbow_;
  SingleAxisJoint l_wrist_;
  SingleAxisJoint r_shoulder_z_;
  DifferentialJoint r_shoulder_xy_;
  DifferentialJoint r_elbow_;
  SingleAxisJoint r_wrist_;
  DifferentialJoint neck_;

  ///////////////////////////
  // Component containers: //

  const std::set<SingleAxisJoint*> saj_set_;        // All SAJs
  const std::map<int, SingleAxisJoint*> saj_map_;   // SUID -> SAJ
  const std::set<DifferentialJoint*> dj_set_;       // All DJs
  const std::map<int, DifferentialJoint*> dj_map_;  // SUID -> DJ
  const std::set<Servo*> servo_set_;                // All Servos
  const std::map<int, Servo*> servo_map_;           // ID -> Servo

  /////////////////////////
  // GF3 Command struct: //

  struct Command {
    uint8_t shots = 0;

    // Oneshot for bit 0
    struct Read {
      uint16_t fileindex = 0;
    } read;

    // Oneshot for bit 1
    struct Write {
      uint16_t fileindex = 0;
    } write;
  } cmd_;

  /////////////////////
  // Configurations: //

  inline static struct Ratios {
    // Shoulders
    const double sz;
    const double sxyd;
    const double sxya;
    // Elbows
    const double ed;
    const double ea;
    // Wrists
    const double wr;
    // Neck
    const double nd;
    const double na;

    Ratios()
        : sz{21.0 * 100.0 / 85.0},
          sxyd{sxya * 94.0 / 100.0},
          sxya{41.0 * 100.0 / 85.0},
          ed{ea * 145.0 / 127.0},
          ea{41.0 * 127.0 / 92.0},
          wr{48.0 / 38.0 * 68.0 / 20.0},
          nd{na * 145.0 / 127.0},
          na{127.0 / 38.0} {}
  } r_{};

  ///////////////////////////////////////////////////
  // ServoUnit Commands serializer & deserializer: //

  friend void to_json(json& j, const GF3& gf3) {
    j["sajs"] = json::array();
    for (const auto* saj : gf3.saj_set_) {
      j["sajs"].push_back(*saj);
    }
    j["djs"] = json::array();
    for (const auto* dj : gf3.dj_set_) {
      j["djs"].push_back(*dj);
    }
  }

  friend void from_json(const json& j, GF3& gf3) {
    for (const auto& saj_json : j.at("sajs")) {
      int suid = saj_json.at("suid").get<int>();
      auto it = std::find_if(
          gf3.saj_set_.begin(), gf3.saj_set_.end(),
          [=](SingleAxisJoint* j) { return j->s_.GetId() == suid; });
      if (it != gf3.saj_set_.end()) {
        from_json(saj_json, **it);
      }
    }
    for (const auto& dj_json : j.at("djs")) {
      int suid = dj_json.at("suid").get<int>();
      auto it = std::find_if(
          gf3.dj_set_.begin(), gf3.dj_set_.end(),
          [=](DifferentialJoint* j) { return j->l_.GetId() == suid; });
      if (it != gf3.dj_set_.end()) {
        from_json(dj_json, **it);
      }
    }
  }
};

}  // namespace gf3
