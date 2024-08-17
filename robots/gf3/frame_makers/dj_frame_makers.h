#pragma once

#include "../servo_units/differential_joint.h"

namespace gf3 {

struct DifferentialJointFrameMakers {
  static std::vector<CanFdFrame> MoveTo(DifferentialJoint*);
  static std::vector<CanFdFrame> Stop(DifferentialJoint*);
  static std::vector<CanFdFrame> Fix(DifferentialJoint*);

  inline static const std::map<DifferentialJoint::Command::Mode,
                               std::vector<CanFdFrame> (*)(DifferentialJoint*)>
      methods{{DifferentialJoint::Command::Mode::MoveTo,
               &DifferentialJointFrameMakers::MoveTo},
              {DifferentialJoint::Command::Mode::Stop,
               &DifferentialJointFrameMakers::Stop},
              {DifferentialJoint::Command::Mode::Fix,
               &DifferentialJointFrameMakers::Fix}};
};

}  // namespace gf3
