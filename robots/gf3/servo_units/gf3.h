#pragma once

#include "differential_joint.h"
#include "single_axis_joint.h"

namespace gf3 {

Servo* servos[14];

class GF3 {
 public:
  SingleAxisJoint l_shoulder_z_, r_shoulder_z_, l_wrist_, r_wrists_;
  DifferentialJoint l_shoulder_xy_, r_shoulder_xy_, l_elbow_, r_elbow_, neck_;
};

}  // namespace gf3
