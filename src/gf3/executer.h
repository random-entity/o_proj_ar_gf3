#pragma once

#include "frame_makers/frame_makers.h"
#include "oneshots/oneshots.h"
#include "rpl_sndrs/udp_rs.h"
#include "servo_units/gf3.h"

namespace gf3 {

class Executer {
 public:
  Executer(GF3& gf3, UdpReplySender& udp_rs) : gf3_{gf3}, udp_rs_{udp_rs} {}

  void Run() {
    // First process GF3-level Oneshots.
    {
      std::lock_guard lock{gf3_.cmd_.mtx};
      GF3Oneshots::Shoot(&gf3_);
    }

    // Set Query ServoID group and Command ServoUnitID group
    static uint8_t group_label = 0;
    if (++group_label == 4) group_label = 0;
    std::set<int> query_group;    // ServoIDs
    std::set<int> command_group;  // ServoUnitIDs
    switch (group_label) {
      case 0:
        query_group = {1, 6, 7, 12};
        command_group = {1, 6, 7, 12};
        break;
      case 1:
        query_group = {2, 3, 4, 5};
        command_group = {2, 4};
        break;
      case 2:
        query_group = {8, 9, 10, 11};
        command_group = {8, 10};
        break;
      case 3:
        query_group = {13, 14};
        command_group = {13};
        break;
      default:
        break;
    }

    // Query and distribute Replies.
    std::vector<CanFdFrame> query_frames;
    std::vector<CanFdFrame> reply_frames;
    for (const auto id : query_group) {
      const auto maybe_servo = utils::SafeAt(gf3_.servo_map_, id);
      if (!maybe_servo) continue;
      auto* servo = maybe_servo.value();
      std::lock_guard lock{servo->mtx_};
      query_frames.push_back(servo->MakeQuery());
    }
    globals::transport->BlockingCycle(&query_frames[0], query_frames.size(),
                                      &reply_frames);
    for (const auto& frame : reply_frames) {
      const auto id = static_cast<int>(frame.source);
      const auto maybe_servo = utils::SafeAt(gf3_.servo_map_, id);
      if (!maybe_servo) continue;
      auto* servo = maybe_servo.value();
      std::lock_guard lock{servo->mtx_};
      servo->SetReply(Query::Parse(frame.data, frame.size));
    }

    bool halt = false;
    for (const auto& s : gf3_.servo_set_) {
      if (static_cast<uint8_t>(
              s->GetReplyAux2PositionUncoiled().extra[1].value) != 0xF) {
        std::cout << "ENCODER INVALIDITY reported from Servo ID " << s->GetId()
                  << std::endl;
        if (s->GetId() == 8 || s->GetId() == 10 || s->GetId() == 11) {
          std::cout << "But temporarily skipping halt for Servo ID "
                    << s->GetId() << std::endl;
          continue;
        }
        halt = true;
      }
    }
    if (halt) {
      udp_rs_.Run();
      for (const auto& s : gf3_.servo_set_) {
        s->SetBrake();
      }
      std::cout << "HALTING program due to ENCODER INVALIDITY" << std::endl;
      while (1);
    }

    // Execute ServoUnit Mode-based Commands.
    std::vector<CanFdFrame> command_frames;
    for (const auto suid : command_group) {
      const auto maybe_saj = utils::SafeAt(gf3_.saj_map_, suid);
      if (maybe_saj) {
        auto* j = maybe_saj.value();
        std::lock_guard lock{j->cmd_.mtx};
        const auto maybe_fm = utils::SafeAt(
            SingleAxisJointFrameMakers::frame_makers, j->cmd_.mode);
        if (maybe_fm) {
          std::lock_guard lock{j->s_.mtx_};
          utils::Merge(command_frames, maybe_fm.value()(j));
        } else {
          std::cout
              << "Mode " << (static_cast<uint8_t>(j->cmd_.mode))
              << " NOT registered to SingleAxisJointFrameMakers::frame_makers."
              << std::endl;
        }

        continue;
      }

      const auto maybe_dj = utils::SafeAt(gf3_.dj_map_, suid);
      if (maybe_dj) {
        auto* j = maybe_dj.value();
        std::lock_guard lock{j->cmd_.mtx};
        const auto maybe_fm = utils::SafeAt(
            DifferentialJointFrameMakers::frame_makers, j->cmd_.mode);
        if (maybe_fm) {
          std::lock_guard lock_l{j->l_.mtx_};
          std::lock_guard lock_r{j->r_.mtx_};
          utils::Merge(command_frames, maybe_fm.value()(j));
        } else {
          std::cout << "Mode " << (static_cast<uint8_t>(j->cmd_.mode))
                    << " NOT registered to "
                       "DifferentialJointFrameMakers::frame_makers."
                    << std::endl;
        }
        continue;
      }
    }

    globals::transport->BlockingCycle(&command_frames[0], command_frames.size(),
                                      nullptr);
  }

 private:
  GF3& gf3_;
  UdpReplySender& udp_rs_;
};

}  // namespace gf3
