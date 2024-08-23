#pragma once

#include "../servo_units/single_axis_joint.h"

namespace gf3 {

struct SingleAxisJointFrameMakers {
  static std::vector<CanFdFrame> Stop(SingleAxisJoint*);
  static std::vector<CanFdFrame> OutPos(SingleAxisJoint*);
  static std::vector<CanFdFrame> OutVel(SingleAxisJoint*);
  static std::vector<CanFdFrame> Fix(SingleAxisJoint*);

  inline static const std::map<SingleAxisJoint::Command::Mode,
                               std::vector<CanFdFrame> (*)(SingleAxisJoint*)>
      frame_makers{{SingleAxisJoint::Command::Mode::Stop, &Stop},
              {SingleAxisJoint::Command::Mode::OutPos, &OutPos},
              {SingleAxisJoint::Command::Mode::OutVel, &OutVel},
              {SingleAxisJoint::Command::Mode::Fix, &Fix}};
};

}  // namespace gf3
