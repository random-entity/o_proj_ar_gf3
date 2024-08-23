#pragma once

#include "../servo_units/differential_joint.h"

namespace gf3 {

struct DifferentialJointFrameMakers {
  static std::vector<CanFdFrame> Stop(DifferentialJoint*);
  static std::vector<CanFdFrame> OutPos(DifferentialJoint*);
  static std::vector<CanFdFrame> OutVel(DifferentialJoint*);
  static std::vector<CanFdFrame> Fix(DifferentialJoint*);

  inline static const std::map<DifferentialJoint::Command::Mode,
                               std::vector<CanFdFrame> (*)(DifferentialJoint*)>
      frame_makers{{DifferentialJoint::Command::Mode::Stop, &Stop},
              {DifferentialJoint::Command::Mode::OutPos, &OutPos},
              {DifferentialJoint::Command::Mode::OutVel, &OutVel},
              {DifferentialJoint::Command::Mode::Fix, &Fix}};
};

}  // namespace gf3
