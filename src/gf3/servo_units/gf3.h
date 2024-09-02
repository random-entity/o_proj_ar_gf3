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
      : l_shoulder_z_{1, 1, r_.sz, mm_.lszmin, mm_.lszmax},
        l_shoulder_xy_{2,
                       3,
                       1,
                       r_.sxyd,
                       r_.sxya,
                       mm_.lsxydmin,
                       mm_.lsxydmax,
                       mm_.lsxyamin,
                       mm_.lsxyamax},
        l_elbow_{4,          5,          1,          r_.ed,     r_.ea,
                 mm_.ledmin, mm_.ledmax, mm_.leamin, mm_.leamax},
        l_wrist_{6, 1, r_.wr, mm_.lwrmin, mm_.lwrmax},
        r_shoulder_z_{7, 4, r_.sz, mm_.rszmin, mm_.rszmax},
        r_shoulder_xy_{8,
                       9,
                       4,
                       r_.sxyd,
                       r_.sxya,
                       mm_.rsxydmin,
                       mm_.rsxydmax,
                       mm_.rsxyamin,
                       mm_.rsxyamax},
        r_elbow_{10,         11,         4,          r_.ed,     r_.ea,
                 mm_.redmin, mm_.redmax, mm_.reamin, mm_.reamax},
        r_wrist_{12, 4, r_.wr, mm_.rwrmin, mm_.rwrmax},
        neck_{13,        14,        3,         r_.nd,    r_.na,
              mm_.ndmin, mm_.ndmax, mm_.namin, mm_.namax},
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
    std::mutex mtx;

    uint8_t oneshots = 0;

    // Oneshot for bit 0.
    struct Read {
      uint16_t fileindex = 0;
    } read;

    // Oneshot for bit 1.
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
  } r_{.sz{21.0 * 100.0 / 85.0},
       .sxyd{41.0 * 100.0 / 85.0 * 94.0 / 100.0},
       .sxya{41.0 * 100.0 / 85.0},
       .ed{41.0 * 127.0 / 92.0 * 145.0 / 127.0},
       .ea{41.0 * 127.0 / 92.0},
       .wr{48.0 / 38.0 * 68.0 / 20.0},
       .nd{127.0 / 38.0 * 145.0 / 127.0},
       .na{127.0 / 38.0}};

  inline static struct MinMaxPos {
    const double lszmin, lszmax;
    const double lsxydmin, lsxydmax;
    const double lsxyamin, lsxyamax;
    const double ledmin, ledmax;
    const double leamin, leamax;
    const double lwrmin, lwrmax;
    const double rszmin, rszmax;
    const double rsxydmin, rsxydmax;
    const double rsxyamin, rsxyamax;
    const double redmin, redmax;
    const double reamin, reamax;
    const double rwrmin, rwrmax;
    const double ndmin, ndmax;
    const double namin, namax;
  } mm_{.lszmin{-0.25},
        .lszmax{0.115},
        .lsxydmin{-0.42},
        .lsxydmax{0.385},
        .lsxyamin{0.0},
        .lsxyamax{0.46},
        .ledmin{-0.475},
        .ledmax{0.33},
        .leamin{-0.08},
        .leamax{0.345},
        .lwrmin{-0.22},
        .lwrmax{0.22},
        .rszmin{-0.15},
        .rszmax{0.215},
        .rsxydmin{-0.42},
        .rsxydmax{0.378},
        .rsxyamin{0.01},
        .rsxyamax{0.45},
        .redmin{-0.45},
        .redmax{0.35},
        .reamin{0.0},
        .reamax{0.395},
        .rwrmin{-0.25},
        .rwrmax{0.213},
        .ndmin{-0.36},
        .ndmax{0.36},
        .namin{-0.22},
        .namax{0.22}};

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
