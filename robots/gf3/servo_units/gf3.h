#pragma once

#include "differential_joint.h"
#include "single_axis_joint.h"

namespace gf3 {

// -------------------------------------------------
// | ServoUnits on GF3:                            |
// |-----------------------------------------------|
// | Type              | Part            | ServoID |
// |                   |                 | L  | R  |
// |-----------------------------------------------|
// | SingleAxisJoint   | LeftShoulderZ   | 1       |
// | DifferentialJoint | LeftShoulderXY  | 2  | 3  |
// | DifferentialJoint | LeftElbow       | 4  | 5  |
// | SingleAxisJoint   | LeftWrist       | 6       |
// | SingleAxisJoint   | RightShoulderZ  | 7       |
// | DifferentialJoint | RightShoulderXY | 8  | 9  |
// | DifferentialJoint | RightElbow      | 10 | 11 |
// | SingleAxisJoint   | RightWrist      | 12      |
// | DifferentialJoint | Neck            | 13 | 14 |
// -------------------------------------------------

class GF3 {
 public:
  GF3()
      : l_shoulder_z_{1, 1, r_.sz, -0.25, 0.25},
        l_shoulder_xy_{2, 3, 1, r_.sxya, r_.sxyd, -0.25, 0.25, -0.25, 0.25},
        l_elbow_{4, 5, 1, r_.ea, r_.ed, -0.25, 0.25, -0.25, 0.25},
        l_wrist_{6, 1, r_.wr, -0.25, 0.25},
        r_shoulder_z_{7, 2, r_.sz, -0.25, 0.25},
        r_shoulder_xy_{8, 9, 2, r_.sxya, r_.sxyd, -0.25, 0.25, -0.25, 0.25},
        r_elbow_{10, 11, 2, r_.ea, r_.ed, -0.25, 0.25, -0.25, 0.25},
        r_wrist_{12, 2, r_.wr, -0.25, 0.25},
        neck_{13, 14, 3, r_.na, r_.nd, -0.25, 0.25, -0.25, 0.25},
        saj_set_{&l_shoulder_z_, &l_wrist_, &r_shoulder_z_, &r_wrist_},
        saj_map_{[&] {
          std::map<int, SingleAxisJoint*> js;
          for (const auto& s : saj_set_) js.emplace(s->s_.GetId(), s);
          return js;
        }()},
        dj_set_{&l_shoulder_xy_, &l_elbow_, &r_shoulder_xy_, &r_elbow_, &neck_},
        dj_map_{[&] {
          std::map<int, DifferentialJoint*> js;
          for (const auto& s : dj_set_) {
            js.emplace(s->l_.GetId(), s);
            js.emplace(s->r_.GetId(), s);
          }
          return js;
        }()},
        dj_lids_{[&] {
          std::set<int> lids;
          for (const auto& s : dj_set_) lids.emplace(s->l_.GetId());
          return lids;
        }()},
        dj_rids_{[&] {
          std::set<int> rids;
          for (const auto& s : dj_set_) rids.emplace(s->r_.GetId());
          return rids;
        }()},
        servos_set_{[&] {
          std::set<Servo*> servos;
          for (const auto& j : saj_set_) servos.emplace(&j->s_);
          for (const auto& j : dj_set_) {
            servos.emplace(&j->l_);
            servos.emplace(&j->r_);
          }
          return servos;
        }()},
        servos_map_{[&] {
          std::map<int, Servo*> map;
          for (const auto& s : servos_set_) map.emplace(s->GetId(), s);
          return map;
        }()},
        ids_{[&] {
          std::set<int> ids;
          for (const auto& s : servos_set_) ids.insert(s->GetId());
          return ids;
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
  const std::set<SingleAxisJoint*> saj_set_;
  const std::map<int, SingleAxisJoint*> saj_map_;
  const std::set<DifferentialJoint*> dj_set_;
  const std::map<int, DifferentialJoint*> dj_map_;
  const std::set<int> dj_lids_;
  const std::set<int> dj_rids_;
  const std::set<Servo*> servos_set_;
  const std::map<int, Servo*> servos_map_;
  const std::set<int> ids_;

  /////////////////////
  // Configurations: //

  const struct {
    // Shoulder
    const double sz = 21.0 * 100.0 / 85.0;
    const double sxya = 41.0 * 100.0 / 85.0;
    const double sxyd = sxya * 94.0 / 100.0;

    // Elbow
    const double ea = 41.0 * 127.0 / 92.0;
    const double ed = ea * 145.0 / 127.0;

    // Wrist
    const double wr = 48.0 / 38.0 * 68.0 / 20.0;

    // Neck
    const double na = 127.0 / 38.0;
    const double nd = na * 145.0 / 127.0;
  } r_;
};

}  // namespace gf3
