#pragma once

#include "frame_makers/frame_makers.h"
#include "oneshots/oneshots.h"
#include "servo_units/gf3.h"

namespace gf3 {

class Executer {
 public:
  Executer(GF3& gf3) : gf3_{gf3} {}

  void Run() {
    // First process GF3-level Oneshots.
    GF3Oneshots::Shoot(&gf3_);

    // Query and distribute Replies.
    std::vector<CanFdFrame> query_frames;
    std::vector<CanFdFrame> reply_frames;
    for (auto* s : gf3_.servo_set_) {
      query_frames.push_back(s->MakeQuery());
    }
    global_transport->BlockingCycle(&query_frames[0], query_frames.size(),
                                    &reply_frames);
    for (const auto& frame : reply_frames) {
      const auto& id = static_cast<int>(frame.source);
      const auto maybe_servo = utils::SafeAt(gf3_.servo_map_, id);
      if (!maybe_servo) continue;
      maybe_servo.value()->SetReply(Query::Parse(frame.data, frame.size));
    }

    // Execute ServoUnit Mode-based Commands.
    std::vector<CanFdFrame> command_frames;
    for (const auto& j : gf3_.saj_set_) {
      try {
        const auto saj_frames =
            SingleAxisJointFrameMakers::methods.at(j->cmd_.mode)(j);
        utils::Merge(command_frames, saj_frames);
      } catch (const std::out_of_range& e) {
        std::cout << "Check if SingleAxisJointFrameMakers::methods "
                     "has all SingleAxisJoint::Command::Mode registered."
                  << std::endl;
        std::cerr << e.what() << std::endl;
      }
    }
    for (const auto& j : gf3_.dj_set_) {
      try {
        const auto dj_frames =
            DifferentialJointFrameMakers::methods.at(j->cmd_.mode)(j);
        utils::Merge(command_frames, dj_frames);
      } catch (const std::out_of_range& e) {
        std::cout << "Check if DifferentialJointFrameMakers::methods "
                     "has all DifferentialJoint::Command::Mode registered."
                  << std::endl;
        std::cerr << e.what() << std::endl;
      }
    }

    global_transport->BlockingCycle(&command_frames[0], command_frames.size(),
                                    nullptr);
  }

 private:
  GF3& gf3_;
};

}  // namespace gf3
