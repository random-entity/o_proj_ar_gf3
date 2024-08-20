#include "zero.h"

using namespace gf3;

int main() {
  GF3 gf3{};
  utils::Zero(gf3);

  // Test.
  std::vector<CanFdFrame> command_frames;
  for (const auto& j : gf3.saj_set_) {
    j->cmd_.mode = SingleAxisJoint::Command::Mode::MoveTo;
    j->cmd_.move_to.target_out = 0.0;
    j->cmd_.move_to.max_trq = 64.0;
    j->cmd_.move_to.max_vel = 64.0;
    j->cmd_.move_to.max_acc = 32.0;
    const auto saj_frames = SingleAxisJointFrameMakers::methods.at(
        SingleAxisJoint::Command::Mode::MoveTo)(j);
    utils::Merge(command_frames, saj_frames);
  }
  for (const auto& j : gf3.dj_set_) {
    j->cmd_.mode = DifferentialJoint::Command::Mode::MoveTo;
    j->cmd_.move_to.target_avg = 0.0;
    j->cmd_.move_to.target_dif = 0.0;
    j->cmd_.move_to.max_trq = 64.0;
    j->cmd_.move_to.max_vel = 64.0;
    j->cmd_.move_to.max_acc = 32.0;
    const auto dj_frames = DifferentialJointFrameMakers::methods.at(
        DifferentialJoint::Command::Mode::MoveTo)(j);
    utils::Merge(command_frames, dj_frames);
  }
  global_transport->BlockingCycle(&command_frames[0], command_frames.size(),
                                  nullptr);

  while (1);

  return 0;
}
